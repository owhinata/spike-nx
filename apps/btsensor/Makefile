include $(APPDIR)/Make.defs

PROGNAME  = btsensor
PRIORITY  = $(CONFIG_APP_BTSENSOR_PRIORITY)
STACKSIZE = $(CONFIG_APP_BTSENSOR_STACKSIZE)
MODULE    = $(CONFIG_APP_BTSENSOR)

MAINSRC = btsensor_main.c

# SPP server (L2CAP + RFCOMM + SDP glue) — Issue #52 Step D.  Named
# btsensor_spp.c to avoid colliding with btstack's src/classic/spp_server.c
# below, which we pull in for the spp_create_sdp_record helper.
CSRCS  += btsensor_spp.c

# IMU sampler / RFCOMM streamer — Issue #52 Step E.
CSRCS  += imu_sampler.c

# Single RFCOMM send arbiter (response queue + frame ring) — Issue #56 B.
CSRCS  += btsensor_tx.c

# BT button (PA0/EXTI0) + LED state machine helpers — Issue #56 Commit C.
CSRCS  += btsensor_button.c btsensor_led.c

# Pulled-in chipset init script (TI v1.4 service pack, eHCILL disabled —
# originally from pybricks, see apps/btsensor/chipset/cc256x_init_script.c).
CSRCS  += chipset/cc256x_init_script.c

# NuttX-side btstack port: run loop, UART wrapper, HAL stubs.
CSRCS  += port/btstack_run_loop_nuttx.c
CSRCS  += port/btstack_uart_nuttx.c

# ---------------------------------------------------------------------------
# btstack integration (Issue #52 Step B/C)
# ---------------------------------------------------------------------------
# BTSTACK_ROOT is resolved relative to nuttx-apps, which lives next to
# spike-nx/libs/btstack on disk.
BTSTACK_ROOT := $(APPDIR)/../libs/btstack

CFLAGS += -I$(APPDIR)/external/btsensor
CFLAGS += -I$(APPDIR)/external/btsensor/port
CFLAGS += -I$(BTSTACK_ROOT)/src
CFLAGS += -I$(BTSTACK_ROOT)/src/classic
CFLAGS += -I$(BTSTACK_ROOT)/chipset/cc256x
CFLAGS += -I$(BTSTACK_ROOT)/platform/embedded

# Silence btstack style warnings that clash with NuttX's strict options.
CFLAGS += -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function
CFLAGS += -Wno-format -Wno-implicit-fallthrough
CFLAGS += -Wno-pointer-sign

# btstack core (from libs/btstack/src/Makefile.inc, minus H5/EM9304 and
# audio/HID/mesh/HSM modules that SPP does not need).
VPATH += :$(BTSTACK_ROOT)/src
CSRCS += ad_parser.c \
         btstack_crypto.c \
         btstack_linked_list.c \
         btstack_memory.c \
         btstack_memory_pool.c \
         btstack_ring_buffer.c \
         btstack_run_loop.c \
         btstack_run_loop_base.c \
         btstack_tlv.c \
         btstack_tlv_none.c \
         btstack_util.c \
         hci.c \
         hci_cmd.c \
         hci_dump.c \
         hci_event.c \
         hci_event_builder.c \
         hci_transport_h4.c \
         l2cap.c \
         l2cap_signaling.c

# btstack Classic (SPP stack: RFCOMM + SDP + link key store).
VPATH += :$(BTSTACK_ROOT)/src/classic
CSRCS += btstack_link_key_db_memory.c \
         rfcomm.c \
         sdp_client.c \
         sdp_client_rfcomm.c \
         sdp_server.c \
         sdp_util.c \
         spp_server.c

# CC2564C chipset driver (init script injection + baud switch).
VPATH += :$(BTSTACK_ROOT)/chipset/cc256x
CSRCS += btstack_chipset_cc256x.c

# Embedded hci_dump backend (routes btstack log_info/log_error + HCI
# packet traces to printf) — re-enable when btsensor_main.c calls
# hci_dump_init() for debugging.
# VPATH += :$(BTSTACK_ROOT)/platform/embedded
# CSRCS += hci_dump_embedded_stdout.c

include $(APPDIR)/Application.mk
