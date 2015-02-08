



#include "ath5k.h"
#include "reg.h"
#include "debug.h"
#include "base.h"


void ath5k_hw_start_rx_dma(struct ath5k_hw *ah)
{
	ATH5K_TRACE(ah->ah_sc);
	ath5k_hw_reg_write(ah, AR5K_CR_RXE, AR5K_CR);
	ath5k_hw_reg_read(ah, AR5K_CR);
}

int ath5k_hw_stop_rx_dma(struct ath5k_hw *ah)
{
	unsigned int i;

	ATH5K_TRACE(ah->ah_sc);
	ath5k_hw_reg_write(ah, AR5K_CR_RXD, AR5K_CR);

	/*
	 * It may take some time to disable the DMA receive unit
	 */
	for (i = 1000; i > 0 &&
			(ath5k_hw_reg_read(ah, AR5K_CR) & AR5K_CR_RXE) != 0;
			i--)
		udelay(10);

	return i ? 0 : -EBUSY;
}

u32 ath5k_hw_get_rxdp(struct ath5k_hw *ah)
{
	return ath5k_hw_reg_read(ah, AR5K_RXDP);
}

void ath5k_hw_set_rxdp(struct ath5k_hw *ah, u32 phys_addr)
{
	ATH5K_TRACE(ah->ah_sc);

	ath5k_hw_reg_write(ah, phys_addr, AR5K_RXDP);
}



int ath5k_hw_start_tx_dma(struct ath5k_hw *ah, unsigned int queue)
{
	u32 tx_queue;

	ATH5K_TRACE(ah->ah_sc);
	AR5K_ASSERT_ENTRY(queue, ah->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is declared inactive */
	if (ah->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return -EIO;

	if (ah->ah_version == AR5K_AR5210) {
		tx_queue = ath5k_hw_reg_read(ah, AR5K_CR);

		/*
		 * Set the queue by type on 5210
		 */
		switch (ah->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_queue |= AR5K_CR_TXE0 & ~AR5K_CR_TXD0;
			break;
		case AR5K_TX_QUEUE_BEACON:
			tx_queue |= AR5K_CR_TXE1 & ~AR5K_CR_TXD1;
			ath5k_hw_reg_write(ah, AR5K_BCR_TQ1V | AR5K_BCR_BDMAE,
					AR5K_BSR);
			break;
		case AR5K_TX_QUEUE_CAB:
			tx_queue |= AR5K_CR_TXE1 & ~AR5K_CR_TXD1;
			ath5k_hw_reg_write(ah, AR5K_BCR_TQ1FV | AR5K_BCR_TQ1V |
				AR5K_BCR_BDMAE, AR5K_BSR);
			break;
		default:
			return -EINVAL;
		}
		/* Start queue */
		ath5k_hw_reg_write(ah, tx_queue, AR5K_CR);
		ath5k_hw_reg_read(ah, AR5K_CR);
	} else {
		/* Return if queue is disabled */
		if (AR5K_REG_READ_Q(ah, AR5K_QCU_TXD, queue))
			return -EIO;

		/* Start queue */
		AR5K_REG_WRITE_Q(ah, AR5K_QCU_TXE, queue);
	}

	return 0;
}

int ath5k_hw_stop_tx_dma(struct ath5k_hw *ah, unsigned int queue)
{
	unsigned int i = 40;
	u32 tx_queue, pending;

	ATH5K_TRACE(ah->ah_sc);
	AR5K_ASSERT_ENTRY(queue, ah->ah_capabilities.cap_queues.q_tx_num);

	/* Return if queue is declared inactive */
	if (ah->ah_txq[queue].tqi_type == AR5K_TX_QUEUE_INACTIVE)
		return -EIO;

	if (ah->ah_version == AR5K_AR5210) {
		tx_queue = ath5k_hw_reg_read(ah, AR5K_CR);

		/*
		 * Set by queue type
		 */
		switch (ah->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_queue |= AR5K_CR_TXD0 & ~AR5K_CR_TXE0;
			break;
		case AR5K_TX_QUEUE_BEACON:
		case AR5K_TX_QUEUE_CAB:
			/* XXX Fix me... */
			tx_queue |= AR5K_CR_TXD1 & ~AR5K_CR_TXD1;
			ath5k_hw_reg_write(ah, 0, AR5K_BSR);
			break;
		default:
			return -EINVAL;
		}

		/* Stop queue */
		ath5k_hw_reg_write(ah, tx_queue, AR5K_CR);
		ath5k_hw_reg_read(ah, AR5K_CR);
	} else {
		/*
		 * Schedule TX disable and wait until queue is empty
		 */
		AR5K_REG_WRITE_Q(ah, AR5K_QCU_TXD, queue);

		/*Check for pending frames*/
		do {
			pending = ath5k_hw_reg_read(ah,
				AR5K_QUEUE_STATUS(queue)) &
				AR5K_QCU_STS_FRMPENDCNT;
			udelay(100);
		} while (--i && pending);

		/* For 2413+ order PCU to drop packets using
		 * QUIET mechanism */
		if (ah->ah_mac_version >= (AR5K_SREV_AR2414 >> 4) &&
		pending){
			/* Set periodicity and duration */
			ath5k_hw_reg_write(ah,
				AR5K_REG_SM(100, AR5K_QUIET_CTL2_QT_PER)|
				AR5K_REG_SM(10, AR5K_QUIET_CTL2_QT_DUR),
				AR5K_QUIET_CTL2);

			/* Enable quiet period for current TSF */
			ath5k_hw_reg_write(ah,
				AR5K_QUIET_CTL1_QT_EN |
				AR5K_REG_SM(ath5k_hw_reg_read(ah,
						AR5K_TSF_L32_5211) >> 10,
						AR5K_QUIET_CTL1_NEXT_QT_TSF),
				AR5K_QUIET_CTL1);

			/* Force channel idle high */
			AR5K_REG_ENABLE_BITS(ah, AR5K_DIAG_SW_5211,
					AR5K_DIAG_SW_CHANEL_IDLE_HIGH);

			/* Wait a while and disable mechanism */
			udelay(200);
			AR5K_REG_DISABLE_BITS(ah, AR5K_QUIET_CTL1,
						AR5K_QUIET_CTL1_QT_EN);

			/* Re-check for pending frames */
			i = 40;
			do {
				pending = ath5k_hw_reg_read(ah,
					AR5K_QUEUE_STATUS(queue)) &
					AR5K_QCU_STS_FRMPENDCNT;
				udelay(100);
			} while (--i && pending);

			AR5K_REG_DISABLE_BITS(ah, AR5K_DIAG_SW_5211,
					AR5K_DIAG_SW_CHANEL_IDLE_HIGH);
		}

		/* Clear register */
		ath5k_hw_reg_write(ah, 0, AR5K_QCU_TXD);
		if (pending)
			return -EBUSY;
	}

	/* TODO: Check for success on 5210 else return error */
	return 0;
}

u32 ath5k_hw_get_txdp(struct ath5k_hw *ah, unsigned int queue)
{
	u16 tx_reg;

	ATH5K_TRACE(ah->ah_sc);
	AR5K_ASSERT_ENTRY(queue, ah->ah_capabilities.cap_queues.q_tx_num);

	/*
	 * Get the transmit queue descriptor pointer from the selected queue
	 */
	/*5210 doesn't have QCU*/
	if (ah->ah_version == AR5K_AR5210) {
		switch (ah->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_reg = AR5K_NOQCU_TXDP0;
			break;
		case AR5K_TX_QUEUE_BEACON:
		case AR5K_TX_QUEUE_CAB:
			tx_reg = AR5K_NOQCU_TXDP1;
			break;
		default:
			return 0xffffffff;
		}
	} else {
		tx_reg = AR5K_QUEUE_TXDP(queue);
	}

	return ath5k_hw_reg_read(ah, tx_reg);
}

int ath5k_hw_set_txdp(struct ath5k_hw *ah, unsigned int queue, u32 phys_addr)
{
	u16 tx_reg;

	ATH5K_TRACE(ah->ah_sc);
	AR5K_ASSERT_ENTRY(queue, ah->ah_capabilities.cap_queues.q_tx_num);

	/*
	 * Set the transmit queue descriptor pointer register by type
	 * on 5210
	 */
	if (ah->ah_version == AR5K_AR5210) {
		switch (ah->ah_txq[queue].tqi_type) {
		case AR5K_TX_QUEUE_DATA:
			tx_reg = AR5K_NOQCU_TXDP0;
			break;
		case AR5K_TX_QUEUE_BEACON:
		case AR5K_TX_QUEUE_CAB:
			tx_reg = AR5K_NOQCU_TXDP1;
			break;
		default:
			return -EINVAL;
		}
	} else {
		/*
		 * Set the transmit queue descriptor pointer for
		 * the selected queue on QCU for 5211+
		 * (this won't work if the queue is still active)
		 */
		if (AR5K_REG_READ_Q(ah, AR5K_QCU_TXE, queue))
			return -EIO;

		tx_reg = AR5K_QUEUE_TXDP(queue);
	}

	/* Set descriptor pointer */
	ath5k_hw_reg_write(ah, phys_addr, tx_reg);

	return 0;
}

int ath5k_hw_update_tx_triglevel(struct ath5k_hw *ah, bool increase)
{
	u32 trigger_level, imr;
	int ret = -EIO;

	ATH5K_TRACE(ah->ah_sc);

	/*
	 * Disable interrupts by setting the mask
	 */
	imr = ath5k_hw_set_imr(ah, ah->ah_imr & ~AR5K_INT_GLOBAL);

	trigger_level = AR5K_REG_MS(ath5k_hw_reg_read(ah, AR5K_TXCFG),
			AR5K_TXCFG_TXFULL);

	if (!increase) {
		if (--trigger_level < AR5K_TUNE_MIN_TX_FIFO_THRES)
			goto done;
	} else
		trigger_level +=
			((AR5K_TUNE_MAX_TX_FIFO_THRES - trigger_level) / 2);

	/*
	 * Update trigger level on success
	 */
	if (ah->ah_version == AR5K_AR5210)
		ath5k_hw_reg_write(ah, trigger_level, AR5K_TRIG_LVL);
	else
		AR5K_REG_WRITE_BITS(ah, AR5K_TXCFG,
				AR5K_TXCFG_TXFULL, trigger_level);

	ret = 0;

done:
	/*
	 * Restore interrupt mask
	 */
	ath5k_hw_set_imr(ah, imr);

	return ret;
}


bool ath5k_hw_is_intr_pending(struct ath5k_hw *ah)
{
	ATH5K_TRACE(ah->ah_sc);
	return ath5k_hw_reg_read(ah, AR5K_INTPEND) == 1 ? 1 : 0;
}

int ath5k_hw_get_isr(struct ath5k_hw *ah, enum ath5k_int *interrupt_mask)
{
	u32 data;

	ATH5K_TRACE(ah->ah_sc);

	/*
	 * Read interrupt status from the Interrupt Status register
	 * on 5210
	 */
	if (ah->ah_version == AR5K_AR5210) {
		data = ath5k_hw_reg_read(ah, AR5K_ISR);
		if (unlikely(data == AR5K_INT_NOCARD)) {
			*interrupt_mask = data;
			return -ENODEV;
		}
	} else {
		/*
		 * Read interrupt status from Interrupt
		 * Status Register shadow copy (Read And Clear)
		 *
		 * Note: PISR/SISR Not available on 5210
		 */
		data = ath5k_hw_reg_read(ah, AR5K_RAC_PISR);
		if (unlikely(data == AR5K_INT_NOCARD)) {
			*interrupt_mask = data;
			return -ENODEV;
		}
	}

	/*
	 * Get abstract interrupt mask (driver-compatible)
	 */
	*interrupt_mask = (data & AR5K_INT_COMMON) & ah->ah_imr;

	if (ah->ah_version != AR5K_AR5210) {
		u32 sisr2 = ath5k_hw_reg_read(ah, AR5K_RAC_SISR2);

		/*HIU = Host Interface Unit (PCI etc)*/
		if (unlikely(data & (AR5K_ISR_HIUERR)))
			*interrupt_mask |= AR5K_INT_FATAL;

		/*Beacon Not Ready*/
		if (unlikely(data & (AR5K_ISR_BNR)))
			*interrupt_mask |= AR5K_INT_BNR;

		if (unlikely(sisr2 & (AR5K_SISR2_SSERR |
					AR5K_SISR2_DPERR |
					AR5K_SISR2_MCABT)))
			*interrupt_mask |= AR5K_INT_FATAL;

		if (data & AR5K_ISR_TIM)
			*interrupt_mask |= AR5K_INT_TIM;

		if (data & AR5K_ISR_BCNMISC) {
			if (sisr2 & AR5K_SISR2_TIM)
				*interrupt_mask |= AR5K_INT_TIM;
			if (sisr2 & AR5K_SISR2_DTIM)
				*interrupt_mask |= AR5K_INT_DTIM;
			if (sisr2 & AR5K_SISR2_DTIM_SYNC)
				*interrupt_mask |= AR5K_INT_DTIM_SYNC;
			if (sisr2 & AR5K_SISR2_BCN_TIMEOUT)
				*interrupt_mask |= AR5K_INT_BCN_TIMEOUT;
			if (sisr2 & AR5K_SISR2_CAB_TIMEOUT)
				*interrupt_mask |= AR5K_INT_CAB_TIMEOUT;
		}

		if (data & AR5K_ISR_RXDOPPLER)
			*interrupt_mask |= AR5K_INT_RX_DOPPLER;
		if (data & AR5K_ISR_QCBRORN) {
			*interrupt_mask |= AR5K_INT_QCBRORN;
			ah->ah_txq_isr |= AR5K_REG_MS(
					ath5k_hw_reg_read(ah, AR5K_RAC_SISR3),
					AR5K_SISR3_QCBRORN);
		}
		if (data & AR5K_ISR_QCBRURN) {
			*interrupt_mask |= AR5K_INT_QCBRURN;
			ah->ah_txq_isr |= AR5K_REG_MS(
					ath5k_hw_reg_read(ah, AR5K_RAC_SISR3),
					AR5K_SISR3_QCBRURN);
		}
		if (data & AR5K_ISR_QTRIG) {
			*interrupt_mask |= AR5K_INT_QTRIG;
			ah->ah_txq_isr |= AR5K_REG_MS(
					ath5k_hw_reg_read(ah, AR5K_RAC_SISR4),
					AR5K_SISR4_QTRIG);
		}

		if (data & AR5K_ISR_TXOK)
			ah->ah_txq_isr |= AR5K_REG_MS(
					ath5k_hw_reg_read(ah, AR5K_RAC_SISR0),
					AR5K_SISR0_QCU_TXOK);

		if (data & AR5K_ISR_TXDESC)
			ah->ah_txq_isr |= AR5K_REG_MS(
					ath5k_hw_reg_read(ah, AR5K_RAC_SISR0),
					AR5K_SISR0_QCU_TXDESC);

		if (data & AR5K_ISR_TXERR)
			ah->ah_txq_isr |= AR5K_REG_MS(
					ath5k_hw_reg_read(ah, AR5K_RAC_SISR1),
					AR5K_SISR1_QCU_TXERR);

		if (data & AR5K_ISR_TXEOL)
			ah->ah_txq_isr |= AR5K_REG_MS(
					ath5k_hw_reg_read(ah, AR5K_RAC_SISR1),
					AR5K_SISR1_QCU_TXEOL);

		if (data & AR5K_ISR_TXURN)
			ah->ah_txq_isr |= AR5K_REG_MS(
					ath5k_hw_reg_read(ah, AR5K_RAC_SISR2),
					AR5K_SISR2_QCU_TXURN);
	} else {
		if (unlikely(data & (AR5K_ISR_SSERR | AR5K_ISR_MCABT
				| AR5K_ISR_HIUERR | AR5K_ISR_DPERR)))
			*interrupt_mask |= AR5K_INT_FATAL;

		/*
		 * XXX: BMISS interrupts may occur after association.
		 * I found this on 5210 code but it needs testing. If this is
		 * true we should disable them before assoc and re-enable them
		 * after a successful assoc + some jiffies.
			interrupt_mask &= ~AR5K_INT_BMISS;
		 */
	}

	/*
	 * In case we didn't handle anything,
	 * print the register value.
	 */
	if (unlikely(*interrupt_mask == 0 && net_ratelimit()))
		ATH5K_PRINTF("ISR: 0x%08x IMR: 0x%08x\n", data, ah->ah_imr);

	return 0;
}

enum ath5k_int ath5k_hw_set_imr(struct ath5k_hw *ah, enum ath5k_int new_mask)
{
	enum ath5k_int old_mask, int_mask;

	old_mask = ah->ah_imr;

	/*
	 * Disable card interrupts to prevent any race conditions
	 * (they will be re-enabled afterwards if AR5K_INT GLOBAL
	 * is set again on the new mask).
	 */
	if (old_mask & AR5K_INT_GLOBAL) {
		ath5k_hw_reg_write(ah, AR5K_IER_DISABLE, AR5K_IER);
		ath5k_hw_reg_read(ah, AR5K_IER);
	}

	/*
	 * Add additional, chipset-dependent interrupt mask flags
	 * and write them to the IMR (interrupt mask register).
	 */
	int_mask = new_mask & AR5K_INT_COMMON;

	if (ah->ah_version != AR5K_AR5210) {
		/* Preserve per queue TXURN interrupt mask */
		u32 simr2 = ath5k_hw_reg_read(ah, AR5K_SIMR2)
				& AR5K_SIMR2_QCU_TXURN;

		if (new_mask & AR5K_INT_FATAL) {
			int_mask |= AR5K_IMR_HIUERR;
			simr2 |= (AR5K_SIMR2_MCABT | AR5K_SIMR2_SSERR
				| AR5K_SIMR2_DPERR);
		}

		/*Beacon Not Ready*/
		if (new_mask & AR5K_INT_BNR)
			int_mask |= AR5K_INT_BNR;

		if (new_mask & AR5K_INT_TIM)
			int_mask |= AR5K_IMR_TIM;

		if (new_mask & AR5K_INT_TIM)
			simr2 |= AR5K_SISR2_TIM;
		if (new_mask & AR5K_INT_DTIM)
			simr2 |= AR5K_SISR2_DTIM;
		if (new_mask & AR5K_INT_DTIM_SYNC)
			simr2 |= AR5K_SISR2_DTIM_SYNC;
		if (new_mask & AR5K_INT_BCN_TIMEOUT)
			simr2 |= AR5K_SISR2_BCN_TIMEOUT;
		if (new_mask & AR5K_INT_CAB_TIMEOUT)
			simr2 |= AR5K_SISR2_CAB_TIMEOUT;

		if (new_mask & AR5K_INT_RX_DOPPLER)
			int_mask |= AR5K_IMR_RXDOPPLER;

		/* Note: Per queue interrupt masks
		 * are set via reset_tx_queue (qcu.c) */
		ath5k_hw_reg_write(ah, int_mask, AR5K_PIMR);
		ath5k_hw_reg_write(ah, simr2, AR5K_SIMR2);

	} else {
		if (new_mask & AR5K_INT_FATAL)
			int_mask |= (AR5K_IMR_SSERR | AR5K_IMR_MCABT
				| AR5K_IMR_HIUERR | AR5K_IMR_DPERR);

		ath5k_hw_reg_write(ah, int_mask, AR5K_IMR);
	}

	/* If RXNOFRM interrupt is masked disable it
	 * by setting AR5K_RXNOFRM to zero */
	if (!(new_mask & AR5K_INT_RXNOFRM))
		ath5k_hw_reg_write(ah, 0, AR5K_RXNOFRM);

	/* Store new interrupt mask */
	ah->ah_imr = new_mask;

	/* ..re-enable interrupts if AR5K_INT_GLOBAL is set */
	if (new_mask & AR5K_INT_GLOBAL) {
		ath5k_hw_reg_write(ah, AR5K_IER_ENABLE, AR5K_IER);
		ath5k_hw_reg_read(ah, AR5K_IER);
	}

	return old_mask;
}

