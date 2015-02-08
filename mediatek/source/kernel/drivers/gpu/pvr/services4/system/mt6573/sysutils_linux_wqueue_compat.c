/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
 ******************************************************************************/

#include <linux/version.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/hardirq.h>
#include <linux/mutex.h>

#include "mach/mt6573_pll.h"

#include "sgxdefs.h"
#include "services_headers.h"
#include "sysinfo.h"
#include "sgxapi_km.h"
#include "sysconfig.h"
#include "sgxinfokm.h"
#include "syslocal.h"

#if !defined(PVR_LINUX_USING_WORKQUEUES)
#error "PVR_LINUX_USING_WORKQUEUES must be defined"
#endif

#define	ONE_MHZ	1000000
#define	HZ_TO_MHZ(m) ((m) / ONE_MHZ)

#define SGX_PARENT_CLOCK "core_ck"

#if !defined(NO_HARDWARE) && \
	defined(LDM_PLATFORM) && \
	defined(SYS_USING_INTERRUPTS) && \
	defined(SYS_OMAP3430_PIN_MEMORY_BUS_CLOCK)
#include <linux/platform_device.h>
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31))
#include <plat/omap-pm.h>
#else
#include <mach/omap-pm.h>
#endif
extern struct platform_device *gpsPVRLDMDev;
#if defined(SGX530) && (SGX_CORE_REV == 125)
#define OMAP_MEMORY_BUS_CLOCK_MAX 800000
#else
#define OMAP_MEMORY_BUS_CLOCK_MAX 664000
#endif
#else 
#undef SYS_OMAP3430_PIN_MEMORY_BUS_CLOCK
#endif 

#define THREED_PLL
//#define CORE_FROM_BUS
#define NEW_API

static IMG_VOID PowerLockWrap(SYS_SPECIFIC_DATA *psSysSpecData)
{
	if (!in_interrupt())
	{
		mutex_lock(&psSysSpecData->sPowerLock);
	}
}

static IMG_VOID PowerLockUnwrap(SYS_SPECIFIC_DATA *psSysSpecData)
{
	if (!in_interrupt())
	{
		mutex_unlock(&psSysSpecData->sPowerLock);
	}
}

PVRSRV_ERROR SysPowerLockWrap(SYS_DATA *psSysData)
{
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;

	PowerLockWrap(psSysSpecData);

	return PVRSRV_OK;
}

IMG_VOID SysPowerLockUnwrap(SYS_DATA *psSysData)
{
	SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;

	PowerLockUnwrap(psSysSpecData);
}

IMG_BOOL WrapSystemPowerChange(SYS_SPECIFIC_DATA *psSysSpecData)
{
	return IMG_TRUE;
}

IMG_VOID UnwrapSystemPowerChange(SYS_SPECIFIC_DATA *psSysSpecData)
{
}

static inline IMG_UINT32 scale_by_rate(IMG_UINT32 val, IMG_UINT32 rate1, IMG_UINT32 rate2)
{
	if (rate1 >= rate2)
	{
		return val * (rate1 / rate2);
	}

	return val / (rate2 / rate1);
}

static inline IMG_UINT32 scale_prop_to_SGX_clock(IMG_UINT32 val, IMG_UINT32 rate)
{
	return scale_by_rate(val, rate, SYS_SGX_CLOCK_SPEED);
}

static inline IMG_UINT32 scale_inv_prop_to_SGX_clock(IMG_UINT32 val, IMG_UINT32 rate)
{
	return scale_by_rate(val, SYS_SGX_CLOCK_SPEED, rate);
}

#if defined(SGX_DYNAMIC_TIMING_INFO)
IMG_VOID SysGetSGXTimingInformation(SGX_TIMING_INFORMATION *psTimingInfo)
{
#if defined(NO_HARDWARE)
    psTimingInfo->ui32CoreClockSpeed = SYS_SGX_CLOCK_SPEED;
#else
    psTimingInfo->ui32CoreClockSpeed = SYS_BUS_CLOCK_SPEED;
#endif

    psTimingInfo->ui32HWRecoveryFreq = scale_prop_to_SGX_clock(SYS_SGX_HWRECOVERY_TIMEOUT_FREQ, SYS_BUS_CLOCK_SPEED);
    psTimingInfo->ui32uKernelFreq = scale_prop_to_SGX_clock(SYS_SGX_PDS_TIMER_FREQ, SYS_BUS_CLOCK_SPEED);
#if defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
    psTimingInfo->bEnableActivePM = IMG_TRUE;
#else
    psTimingInfo->bEnableActivePM = IMG_FALSE;
#endif
    psTimingInfo->ui32ActivePowManLatencyms = SYS_SGX_ACTIVE_POWER_LATENCY_MS;
}
#endif

PVRSRV_ERROR EnableSGXClocks(SYS_DATA *psSysData)
{
#if !defined(NO_HARDWARE)

    SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;

    if (atomic_read(&psSysSpecData->sSGXClocksEnabled) != 0)
    {
        return PVRSRV_OK;
    }

    PVR_DPF((PVR_DBG_MESSAGE, "EnableSGXClocks: Enabling SGX Clocks"));

#ifdef NEW_API

    hwEnableClock(MT65XX_PDN_MM_GMC2, "MFG");
    hwEnableClock(MT65XX_PDN_MM_GMC2_E, "MFG");

#ifdef THREED_PLL
    DRV_WriteReg32(MMSYS2_CONFIG_BASE + 0x0500, 0x0); // select 3d pll
    hwSetClockType(MT65XX_PDN_MM_MFG_CORE, F3D_CK, "MFG");
#else
    #ifdef CORE_FROM_BUS
        DRV_WriteReg32(MMSYS2_CONFIG_BASE + 0x0500, 0x2); // select bus
        hwSetClockType(MT65XX_PDN_MM_MFG_CORE, FWMA_CK, "MFG");
    #else
        DRV_WriteReg32(MMSYS2_CONFIG_BASE + 0x0500, 0x1); // select mem
        hwSetClockType(MT65XX_PDN_MM_MFG_CORE, FEMI_CK, "MFG");
    #endif
#endif

#else //#ifdef NEW_API

#ifdef THREED_PLL
    hwEnablePLL(MT65XX_3D_PLL, "MFG");
    DRV_WriteReg32(MMSYS2_CONFIG_BASE + 0x0500, 0x0); // select 3d pll
#else
    #ifdef CORE_FROM_BUS
        DRV_WriteReg32(MMSYS2_CONFIG_BASE + 0x0500, 0x2); // select bus
    #else
        DRV_WriteReg32(MMSYS2_CONFIG_BASE + 0x0500, 0x1); // select mem
    #endif
#endif

#endif //#ifdef NEW_API

    hwDisableClock(MT65XX_PDN_MM_MFG_SYS, "MFG_t");
    hwEnableClock(MT65XX_PDN_MM_MFG_CORE, "MFG");
    hwEnableClock(MT65XX_PDN_MM_MFG_MEM, "MFG");
    hwEnableClock(MT65XX_PDN_MM_MFG_SYS, "MFG");
    hwEnableClock(MT65XX_PDN_MM_MFG_SYS, "MFG_t");

    atomic_set(&psSysSpecData->sSGXClocksEnabled, 1);

#else

    PVR_UNREFERENCED_PARAMETER(psSysData);

#endif

	return PVRSRV_OK;
}


IMG_VOID DisableSGXClocks(SYS_DATA *psSysData)
{
#if !defined(NO_HARDWARE)

    SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;

    if (atomic_read(&psSysSpecData->sSGXClocksEnabled) == 0)
    {
        return;
    }

    PVR_DPF((PVR_DBG_MESSAGE, "DisableSGXClocks: Disabling SGX Clocks"));

    hwDisableClock(MT65XX_PDN_MM_MFG_SYS, "MFG");
    hwDisableClock(MT65XX_PDN_MM_MFG_MEM, "MFG");
    hwDisableClock(MT65XX_PDN_MM_MFG_CORE, "MFG");

#ifdef NEW_API
    hwDisableClock(MT65XX_PDN_MM_GMC2_E, "MFG");
    hwDisableClock(MT65XX_PDN_MM_GMC2, "MFG");
#endif

    atomic_set(&psSysSpecData->sSGXClocksEnabled, 0);

#else

    PVR_UNREFERENCED_PARAMETER(psSysData);

#endif
}

PVRSRV_ERROR EnableSystemClocks(SYS_DATA *psSysData)
{
    SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;

    PVR_TRACE(("EnableSystemClocks: Enabling System Clocks"));

    if (!psSysSpecData->bSysClocksOneTimeInit)
    {
        mutex_init(&psSysSpecData->sPowerLock);

        atomic_set(&psSysSpecData->sSGXClocksEnabled, 0);

        psSysSpecData->bSysClocksOneTimeInit = IMG_TRUE;
    }

    return PVRSRV_OK;
}

IMG_VOID DisableSystemClocks(SYS_DATA *psSysData)
{
    SYS_SPECIFIC_DATA *psSysSpecData = (SYS_SPECIFIC_DATA *) psSysData->pvSysSpecificData;

    PVR_TRACE(("DisableSystemClocks: Disabling System Clocks"));

    DisableSGXClocks(psSysData);

    if (psSysSpecData->bSysClocksOneTimeInit == IMG_TRUE)
    {
        psSysSpecData->bSysClocksOneTimeInit = IMG_FALSE;
    }
}

IMG_VOID EnableSGXClocks_t(SYS_DATA *psSysData)
{
    hwEnableClock(MT65XX_PDN_MM_MFG_SYS, "MFG_t");
}

IMG_VOID DisableSGXClocks_t(SYS_DATA *psSysData)
{
    hwDisableClock(MT65XX_PDN_MM_MFG_SYS, "MFG_t");
}
