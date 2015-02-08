RESMODE=static

# Resource shortname
RESNAME=fortune_resources

RESLDFLAGS=
# Don't call udata_setAppData unless we are linked with the data
RESCPPFLAGS=-DUFORTUNE_NOSETAPPDATA
CHECK_VARS= ICU_DATA=$(RESDIR)

# DLL and static modes are identical here
ifeq ($(RESMODE),dll)
RESLDFLAGS= -L$(RESDIR) -l$(RESNAME)
RESCPPFLAGS=
CHECK_VARS=
endif

ifeq ($(RESMODE),static)
RESLDFLAGS= -L$(RESDIR) -l$(RESNAME)
RESCPPFLAGS=
CHECK_VARS=
endif

