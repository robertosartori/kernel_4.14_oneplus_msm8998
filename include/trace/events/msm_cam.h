/* Copyright (c) 2016, 2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM msm_cam

#if !defined(_TRACE_MSM_VFE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MSM_VFE_H

#include <../drivers/media/platform/msm/camera_v2/isp/msm_isp.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#define STRING_LEN 80


TRACE_EVENT(msm_cam_string,
	TP_PROTO(const char *str),
	TP_ARGS(str),
	TP_STRUCT__entry(
		__array(char, str, STRING_LEN)
	),
	TP_fast_assign(
		strlcpy(__entry->str, str, STRING_LEN);
	),
	TP_printk("msm_cam: %s", __entry->str)
);

TRACE_EVENT(msm_cam_isp_status_dump,
	TP_PROTO(char *event, uint32_t vfe_id, uint32_t frame_id,
		uint32_t irq_status0, uint32_t irq_status1,
		uint32_t dual_irq_status),
	TP_ARGS(event, vfe_id, frame_id, irq_status0,
		irq_status1, dual_irq_status),
	TP_STRUCT__entry(
		__field(char *, event)
		__field(unsigned int, vfe_id)
		__field(unsigned int, frame_id)
		__field(unsigned int, irq_status0)
		__field(unsigned int, irq_status1)
		__field(unsigned int, dual_irq_status)
	),
	TP_fast_assign(
		__entry->event = event;
		__entry->vfe_id = vfe_id;
		__entry->frame_id = frame_id;
		__entry->irq_status0 = irq_status0;
		__entry->irq_status1 = irq_status1;
		__entry->dual_irq_status = dual_irq_status;
	),
	TP_printk("%s vfe %d, frame %d, irq_st0 %x, irq_st1 %x dual_irq %x",
		__entry->event,
		__entry->vfe_id,
		__entry->frame_id,
		__entry->irq_status0,
		__entry->irq_status1,
		__entry->dual_irq_status
	)
);

#endif /* _MSM_CAM_TRACE_H */
/* This part must be outside protection */
#include <trace/define_trace.h>
