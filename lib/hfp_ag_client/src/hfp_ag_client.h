#include <stdio.h>
#include "app_hf_msg_prs.h"
#include "app_hf_msg_set.h"
#include "bt_app_core.h"
#include "bt_app_hf.h"
#include "gpio_pcm_config.h"

#define BT_HF_AG_TAG    "HF_AG_DEMO_MAIN"

static void bt_hf_hdl_stack_evt(uint16_t event, void *p_param);

void hfp_ag_init(void);

