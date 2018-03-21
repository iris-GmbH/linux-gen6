ifeq ($(CONFIG_MACH_SC573_EZKIT),y)
zreladdr-y	+= 0x82008000
params_phys-y	:= 0x82000100
endif
ifeq ($(CONFIG_MACH_SC572_GEN6),y)
zreladdr-y	+= 0x82008000
params_phys-y	:= 0x82000100
endif
ifeq ($(CONFIG_MACH_SC573_GEN6),y)
zreladdr-y	+= 0x82008000
params_phys-y	:= 0x82000100
endif
