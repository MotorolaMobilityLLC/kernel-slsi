/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __VERTEX_COMMON_TYPE_H__
#define __VERTEX_COMMON_TYPE_H__

#define MAX_INPUT_NUM		(8)
#define MAX_OUTPUT_NUM		(8)
#define MAX_PLANE_NUM		(3)

enum vertex_common_addr_type {
	VERTEX_COMMON_V_ADDR,
	VERTEX_COMMON_DV_ADDR,
	VERTEX_COMMON_FD,
};

enum vertex_common_mem_attr {
	VERTEX_COMMON_CACHEALBE,
	VERTEX_COMMON_NON_CACHEALBE,
};

enum vertex_common_mem_type {
	VERTEX_COMMON_MEM_ION,
	VERTEX_COMMON_MEM_MALLOC,
	VERTEX_COMMON_MEM_ASHMEM
};

struct vertex_common_mem {
	int		fd;
	unsigned int	iova;
	unsigned int	size;
	unsigned int	offset;
	unsigned char	addr_type;
	unsigned char	mem_attr;
	unsigned char	mem_type;
	unsigned char	reserved[5];
};

union vertex_common_global_id {
	struct {
		uint32_t head :2;
		uint32_t target :2;
		uint32_t model_id :12;
		uint32_t msg_id :16;
	} head;
	unsigned int num;
};

struct vertex_common_graph_info {
	struct vertex_common_mem		graph;
	struct vertex_common_mem		temp_buf;
	struct vertex_common_mem		weight;
	struct vertex_common_mem		bias;
	unsigned int			gid;
	unsigned int			user_para_size;
	//unsigned char			user_para[0] // varialbe size
};

struct vertex_common_execute_info {
	unsigned int			gid;
	unsigned int			macro_sg_offset;
	unsigned int			num_input;
	unsigned int			num_output;
	struct vertex_common_mem	input[MAX_INPUT_NUM][MAX_PLANE_NUM];
	struct vertex_common_mem	output[MAX_OUTPUT_NUM][MAX_PLANE_NUM];
	unsigned int			user_para_size;
	unsigned int			reserved;
	//unsigned char			user_para[0] // varialbe size
};

#endif
