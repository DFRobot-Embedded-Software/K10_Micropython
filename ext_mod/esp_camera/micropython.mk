CAMERA_MOD_DIR := $(USERMOD_DIR)
#SRC_USERMOD += $(addprefix $(CAMERA_MOD_DIR)/, modcamera.c)
SRC_USERMOD += $(CEXAMPLE_MOD_DIR)/modcamera.c

CFLAGS_USERMOD += -I$(CAMERA_MOD_DIR)
