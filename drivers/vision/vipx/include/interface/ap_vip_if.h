/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * Version history
 * 2017.06.27 : Initial draft
 * 2017.08.25 : Released
 * 2017.10.12 : Add BOOTUP_RSP
 */

#ifndef AP_VIP_IF_H_
#define AP_VIP_IF_H_

#ifdef __cplusplus
extern "C" {
#endif

// temp for previous host ver
#if 0
#define AP_FOCAL_ZONE_SETTING 0
#else
#define AP_FOCAL_ZONE_SETTING 1
#endif
/**
    Typedefs
*/
typedef u32 dram_addr_t;
typedef u32 graph_id_t;
typedef u32 job_id_t;

/**
  Message type
*/
enum
{
	BOOTUP_RSP = 1,
	INIT_REQ,
	INIT_RSP,
	CREATE_GRAPH_REQ,
	CREATE_GRAPH_RSP,
	SET_GRAPH_REQ,
	SET_GRAPH_RSP,
	INVOKE_GRAPH_REQ,
	INVOKE_GRAPH_ACK,
	INVOKE_GRAPH_RSP,
	DESTROY_GRAPH_REQ,
	DESTROY_GRAPH_RSP,
	ABORT_GRAPH_REQ,
	ABORT_GRAPH_RSP,
	POWER_DOWN_REQ,
	POWER_DOWN_RSP,
	MAX_MSG_TYPE,
};

typedef u32 vipx_msgid_e;

/**
    Constants
*/
enum
{
// temp for previous host ver
#if 0
MAX_NUM_OF_INPUTS = 16,
#else
	MAX_NUM_OF_INPUTS = 24,
#endif
#if 0 
	MAX_NUM_OF_OUTPUTS = 12,
#else
	MAX_NUM_OF_OUTPUTS = 8,
#endif
// temp for previous host ver
#if 0
	MAX_NUM_OF_USER_PARAMS = 8,
#else
	MAX_NUM_OF_USER_PARAMS = 180,
#endif
};

/**
    Error indications
*/
enum
{
	SUCCESS = 0,
	ERROR_INVALID_GRAPH_ID = -1,
	ERROR_EXEC_PARAM_CORRUPTION = -2,
	ERROR_GRAPH_DATA_CORRUPTION = -3,
	ERROR_COMPILED_GRAPH_ENABLED = -4,
	ERROR_INVALID_MSG_TYPE = -5,
	ERROR_UNKNOWN = -6,
	ERROR_INVALID = -7,
	ERROR_MBX_BUSY = -8,
};


typedef s32 vipx_error_e;

/**
    Predefined builtin algorithms.
*/

enum
{
	SCENARIO_DE = 1,
	SCENARIO_SDOF_FULL,
	SCENARIO_DE_SDOF,
	SCENARIO_DE_CAPTURE,
	SCENARIO_SDOF_CAPTURE,
	SCENARIO_DE_SDOF_CAPTURE,
	SCENARIO_GDC,
	SCENARIO_ENF,
	SCENARIO_ENF_UV,
	SCENARIO_ENF_YUV,
	SCENARIO_BLEND,
	SCENARIO_AP_MAX,
};

#define SCENARIO_MAX SCENARIO_AP_MAX

typedef u32 vipx_builtin_graph_id_e;

typedef struct
{
    vipx_error_e error;
} __attribute__((__packed__)) vipx_bootup_rsp_t;

typedef struct
{
    dram_addr_t p_cc_heap;      //< pointer to CC heap region
    u32 sz_cc_heap;          //< size of CC heap region
} __attribute__((__packed__)) vipx_init_req_t;

typedef struct
{
    vipx_error_e error;
} __attribute__((__packed__)) vipx_init_rsp_t;

typedef struct
{
    dram_addr_t p_graph;        //< pointer to compiled graph
    u32 sz_graph;            //< size of compiled graph
} __attribute__((__packed__)) vipx_create_graph_req_t;

typedef struct
{
    vipx_error_e error;
    graph_id_t graph_id;        //< graph id that should be used for invocation
} __attribute__((__packed__)) vipx_create_graph_rsp_t;

typedef struct
{
    graph_id_t graph_id;

    dram_addr_t p_lll_bin;                //< pointer to VIP binary in DRAM
    u32 sz_lll_bin;                    //< size of VIP binary in DRAM

    int num_inputs;                       //< number of inputs for the graph
    int num_outputs;                      //< number of outputs for the graph

    u32 input_width[MAX_NUM_OF_INPUTS];       //< array of input width sizes
    u32 input_height[MAX_NUM_OF_INPUTS];      //< array of input height sizes
    u32 input_depth[MAX_NUM_OF_INPUTS];       //< array of input depth sizes

    u32 output_width[MAX_NUM_OF_OUTPUTS];     //< array of output width sizes
    u32 output_height[MAX_NUM_OF_OUTPUTS];    //< array of output height sizes
    u32 output_depth[MAX_NUM_OF_OUTPUTS];     //< array of output depth sizes

    dram_addr_t p_temp;                   //< pointer to DRAM area for temporary buffers
    u32 sz_temp;                       //< size of temporary buffer area
} __attribute__((__packed__)) vipx_set_graph_req_t;

typedef struct
{
    vipx_error_e error;
} __attribute__((__packed__)) vipx_set_graph_rsp_t;

typedef struct
{
    graph_id_t graph_id;                 //< graph id that should be used for invocation
    int num_inputs;                      //< number of inputs for the graph
    int num_outputs;                     //< number of outputs for the graph
    dram_addr_t p_input[MAX_NUM_OF_INPUTS];     //< array of input buffers
    dram_addr_t p_output[MAX_NUM_OF_OUTPUTS];   //< array of output buffers
#if AP_FOCAL_ZONE_SETTING
	u32 user_params[MAX_NUM_OF_USER_PARAMS];    //< array of user parameters
#endif
} __attribute__((__packed__)) vipx_invoke_graph_req_t;

typedef struct
{
    vipx_error_e error;
    job_id_t job_id;
} __attribute__((__packed__)) vipx_invoke_graph_ack_t;

typedef struct
{
    vipx_error_e error;
    job_id_t job_id;
} __attribute__((__packed__)) vipx_invoke_graph_rsp_t;

typedef struct
{
    job_id_t job_id;
} __attribute__((__packed__)) vipx_abort_graph_req_t;

typedef struct
{
    vipx_error_e error;
} __attribute__((__packed__)) vipx_abort_graph_rsp_t;

typedef struct
{
    graph_id_t graph_id;
} __attribute__((__packed__)) vipx_destroy_graph_req_t;

typedef struct
{
    vipx_error_e error;
} __attribute__((__packed__)) vipx_destroy_graph_rsp_t;

typedef struct
{
	uint32_t valid;
} __attribute__((__packed__)) vipx_powerdown_req_t;

typedef struct
{
    vipx_error_e error;
} __attribute__((__packed__)) vipx_powerdown_rsp_t;


union __attribute__((__packed__)) vipx_messages_u
{
    vipx_bootup_rsp_t              bootup_rsp;
    vipx_init_req_t                init_req;
    vipx_init_rsp_t                init_rsp;
    vipx_create_graph_req_t        create_graph_req;
    vipx_create_graph_rsp_t        create_graph_rsp;
    vipx_set_graph_req_t           set_graph_req;
    vipx_set_graph_rsp_t           set_graph_rsp;
    vipx_invoke_graph_req_t        invoke_graph_req;
    vipx_invoke_graph_ack_t        invoke_graph_ack;
    vipx_invoke_graph_rsp_t        invoke_graph_rsp;
    vipx_destroy_graph_req_t       destroy_graph_req;
    vipx_destroy_graph_rsp_t       destroy_graph_rsp;
    vipx_abort_graph_req_t         abort_graph_req;
    vipx_abort_graph_rsp_t         abort_graph_rsp;
    vipx_powerdown_req_t           powerdown_req;
    vipx_powerdown_rsp_t           powerdown_rsp;
};

typedef struct
{
 // TODO: valid flag to be removed.
    uint32_t valid;
    uint32_t transId;
    vipx_msgid_e type;
    union vipx_messages_u msg_u;
} __attribute__((__packed__)) vipx_msg_t;

#ifdef __cplusplus
}
#endif

#endif // AP_VIP_IF_H_
