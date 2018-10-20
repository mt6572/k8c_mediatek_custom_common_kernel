
obj-y += mtk_tpd.o
obj-y += tpd_button.o
obj-y += tpd_calibrate.o
obj-y += tpd_debug.o
obj-y += tpd_default.o
obj-y += tpd_init.o
obj-y += tpd_misc.o
obj-y += tpd_setting.o
obj-y += mtk_gslX680.o

obj-y += gsl_point_id.o
$(obj)/gsl_point_id.o: $(srctree)/$(obj)/gsl_point_id
	cp $(srctree)/$(obj)/gsl_point_id $(obj)/gsl_point_id.o

#include $(MTK_PATH_BUILD)/common.mk
# path := $(if $(filter yes,$(KBUILD_OUTPUT_SUPPORT)),$(srctree)/$(obj),$(obj))
# obj-y := $(patsubst %.c,%.o,$(subst $(path)/,,$(call wildcard2,$(path)/*.c)))
#obj-y += gsl_point_id
