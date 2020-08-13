

#include <firmament.h>
#include <string.h>
#include "task/task_logger.h"
#include "module/fs_manager/fs_manager.h"
#include "module/log/blog.h"
#include "module/log/ulog.h"
#include "module/console/console.h"

#define TAG     "BLog"

/* BLog element define */
blog_elem_t IMU_elems[] = {
	BLOG_ELEMENT("timestamp_ms", BLOG_UINT32),
	BLOG_ELEMENT("gyr_x_radPs_B", BLOG_FLOAT),
	BLOG_ELEMENT("gyr_y_radPs_B", BLOG_FLOAT),
	BLOG_ELEMENT("gyr_z_radPs_B", BLOG_FLOAT),
	BLOG_ELEMENT("acc_x_mPs2_B", BLOG_FLOAT),
	BLOG_ELEMENT("acc_y_mPs2_B", BLOG_FLOAT),
	BLOG_ELEMENT("acc_z_mPs2_B", BLOG_FLOAT),
};

blog_elem_t MAG_elems[] = {
	BLOG_ELEMENT("timestamp_ms", BLOG_UINT32),
	BLOG_ELEMENT("mag_x_ga_B", BLOG_FLOAT),
	BLOG_ELEMENT("mag_y_ga_B", BLOG_FLOAT),
	BLOG_ELEMENT("mag_z_ga_B", BLOG_FLOAT),
};

blog_elem_t BARO_elems[] = {
	BLOG_ELEMENT("timestamp_ms", BLOG_UINT32),
	BLOG_ELEMENT("pressure_pa", BLOG_FLOAT),
	BLOG_ELEMENT("temperature_deg", BLOG_FLOAT),
};

blog_elem_t uBlox_PVT_elems[] = {
	BLOG_ELEMENT("timestamp_ms", BLOG_UINT32),
	BLOG_ELEMENT("iTOW", BLOG_UINT32),
	BLOG_ELEMENT("year", BLOG_UINT16),
	BLOG_ELEMENT("month", BLOG_UINT8),
	BLOG_ELEMENT("day", BLOG_UINT8),
	BLOG_ELEMENT("hour", BLOG_UINT8),
	BLOG_ELEMENT("min", BLOG_UINT8),
	BLOG_ELEMENT("sec", BLOG_UINT8),
	BLOG_ELEMENT("valid", BLOG_UINT8),
	BLOG_ELEMENT("tAcc", BLOG_UINT32),
	BLOG_ELEMENT("nano", BLOG_INT32),
	BLOG_ELEMENT("fixType", BLOG_UINT8),
	BLOG_ELEMENT("flags", BLOG_UINT8),
	BLOG_ELEMENT("reserved1", BLOG_UINT8),
	BLOG_ELEMENT("numSV", BLOG_UINT8),
	BLOG_ELEMENT("lon", BLOG_INT32),
	BLOG_ELEMENT("lat", BLOG_INT32),
	BLOG_ELEMENT("height", BLOG_INT32),
	BLOG_ELEMENT("hMSL", BLOG_INT32),
	BLOG_ELEMENT("hAcc", BLOG_UINT32),
	BLOG_ELEMENT("vAcc", BLOG_UINT32),
	BLOG_ELEMENT("velN", BLOG_INT32),
	BLOG_ELEMENT("velE", BLOG_INT32),
	BLOG_ELEMENT("velD", BLOG_INT32),
	BLOG_ELEMENT("gSpeed", BLOG_INT32),
	BLOG_ELEMENT("headMot", BLOG_INT32),
	BLOG_ELEMENT("sAcc", BLOG_UINT32),
	BLOG_ELEMENT("headAcc", BLOG_UINT32),
	BLOG_ELEMENT("pDOP", BLOG_UINT16),
	BLOG_ELEMENT("reserved2", BLOG_UINT16),
};

blog_elem_t Pilot_Cmd_elems[] = {
	BLOG_ELEMENT("timestamp", BLOG_UINT32),
	BLOG_ELEMENT("ls_lr", BLOG_FLOAT),
	BLOG_ELEMENT("ls_ud", BLOG_FLOAT),
    BLOG_ELEMENT("rs_lr", BLOG_FLOAT),
    BLOG_ELEMENT("rs_ud", BLOG_FLOAT),
    BLOG_ELEMENT("mode", BLOG_UINT32),
    BLOG_ELEMENT("command_1", BLOG_UINT32),
    BLOG_ELEMENT("command_2", BLOG_UINT32),
};

blog_elem_t INS_Out_elems[] = {
	BLOG_ELEMENT("timestamp", BLOG_UINT32),
	BLOG_ELEMENT("phi", BLOG_FLOAT),
	BLOG_ELEMENT("theta", BLOG_FLOAT),
	BLOG_ELEMENT("psi", BLOG_FLOAT),
	BLOG_ELEMENT_VEC("quat", BLOG_FLOAT, 4),
	BLOG_ELEMENT("p", BLOG_FLOAT),
	BLOG_ELEMENT("q", BLOG_FLOAT),
	BLOG_ELEMENT("r", BLOG_FLOAT),
	BLOG_ELEMENT("ax", BLOG_FLOAT),
	BLOG_ELEMENT("ay", BLOG_FLOAT),
	BLOG_ELEMENT("az", BLOG_FLOAT),
	BLOG_ELEMENT("vn", BLOG_FLOAT),
	BLOG_ELEMENT("ve", BLOG_FLOAT),
	BLOG_ELEMENT("vd", BLOG_FLOAT),
    BLOG_ELEMENT("reserved", BLOG_FLOAT),
	BLOG_ELEMENT("lon", BLOG_DOUBLE),
	BLOG_ELEMENT("lat", BLOG_DOUBLE),
	BLOG_ELEMENT("alt", BLOG_DOUBLE),
	BLOG_ELEMENT("x_R", BLOG_FLOAT),
	BLOG_ELEMENT("y_R", BLOG_FLOAT),
	BLOG_ELEMENT("h_R", BLOG_FLOAT),
    BLOG_ELEMENT("h_AGL", BLOG_FLOAT),
	BLOG_ELEMENT("flag", BLOG_UINT32),
	BLOG_ELEMENT("status", BLOG_UINT32),
};

blog_elem_t FMS_Out_elems[] = {
	BLOG_ELEMENT("timestamp", BLOG_UINT32),
	BLOG_ELEMENT("p_cmd", BLOG_FLOAT),
	BLOG_ELEMENT("q_cmd", BLOG_FLOAT),
    BLOG_ELEMENT("r_cmd", BLOG_FLOAT),
    BLOG_ELEMENT("phi_cmd", BLOG_FLOAT),
    BLOG_ELEMENT("theta_cmd", BLOG_FLOAT),
    BLOG_ELEMENT("psi_rate_cmd", BLOG_FLOAT),
    BLOG_ELEMENT("u_cmd", BLOG_FLOAT),
    BLOG_ELEMENT("v_cmd", BLOG_FLOAT),
    BLOG_ELEMENT("w_cmd", BLOG_FLOAT),
    BLOG_ELEMENT("throttle_cmd", BLOG_UINT32),
    BLOG_ELEMENT_VEC("actuator_cmd", BLOG_UINT16, 16),
    BLOG_ELEMENT("state", BLOG_UINT8),
    BLOG_ELEMENT("mode", BLOG_UINT8),
    BLOG_ELEMENT("reset", BLOG_UINT8),
    BLOG_ELEMENT("reserved", BLOG_UINT8),
};

blog_elem_t Control_Out_elems[] = {
    BLOG_ELEMENT("timestamp", BLOG_UINT32),
	BLOG_ELEMENT_VEC("actuator_cmd", BLOG_UINT16, 16),
};

#ifdef FMT_USING_HIL
blog_elem_t Plant_States_elems[] = {
    BLOG_ELEMENT("timestamp", BLOG_UINT32),
	BLOG_ELEMENT("phi", BLOG_FLOAT),
    BLOG_ELEMENT("theta", BLOG_FLOAT),
    BLOG_ELEMENT("psi", BLOG_FLOAT),
    BLOG_ELEMENT("rot_x_B", BLOG_FLOAT),
    BLOG_ELEMENT("rot_y_B", BLOG_FLOAT),
    BLOG_ELEMENT("rot_z_B", BLOG_FLOAT),
    BLOG_ELEMENT("acc_x_O", BLOG_FLOAT),
    BLOG_ELEMENT("acc_y_O", BLOG_FLOAT),
    BLOG_ELEMENT("acc_z_O", BLOG_FLOAT),
    BLOG_ELEMENT("vel_x_O", BLOG_FLOAT),
    BLOG_ELEMENT("vel_y_O", BLOG_FLOAT),
    BLOG_ELEMENT("vel_z_O", BLOG_FLOAT),
    BLOG_ELEMENT("x_R", BLOG_FLOAT),
    BLOG_ELEMENT("y_R", BLOG_FLOAT),
    BLOG_ELEMENT("h_R", BLOG_FLOAT),
    BLOG_ELEMENT("lon", BLOG_DOUBLE),
    BLOG_ELEMENT("lat", BLOG_DOUBLE),
    BLOG_ELEMENT("alt", BLOG_DOUBLE),
    BLOG_ELEMENT("lon_ref", BLOG_DOUBLE),
    BLOG_ELEMENT("lat_ref", BLOG_DOUBLE),
    BLOG_ELEMENT("alt_ref", BLOG_DOUBLE),
};
#endif

/* BLog bus define */
blog_bus_t _blog_bus[] = {
	BLOG_BUS("IMU", BLOG_IMU_ID, IMU_elems),
	BLOG_BUS("MAG", BLOG_MAG_ID, MAG_elems),
	BLOG_BUS("BARO", BLOG_BARO_ID, BARO_elems),
	BLOG_BUS("uBlox_PVT", BLOG_GPS_ID, uBlox_PVT_elems),
    BLOG_BUS("Pilot_Cmd", BLOG_PILOT_CMD_ID, Pilot_Cmd_elems),
	BLOG_BUS("INS_Out", BLOG_INS_OUT_ID, INS_Out_elems),
    BLOG_BUS("FMS_Out", BLOG_FMS_OUT_ID, FMS_Out_elems),
    BLOG_BUS("Control_Out", BLOG_CONTROL_OUT_ID, Control_Out_elems),
#ifdef FMT_USING_HIL
    BLOG_BUS("Plant_States", BLOG_PLANT_STATE_ID, Plant_States_elems),
#endif
};

typedef struct {
	uint32_t total_msg;
	uint32_t lost_msg;
} blog_stat_t;

struct fmt_blog {
	int                 fid;
	uint8_t             file_open;
	char                file_name[50];
	uint8_t             log_status;
	blog_header_t       header;
	blog_buffer_t       buffer;
	blog_stat_t         monitor[sizeof(_blog_bus) / sizeof(blog_bus_t)];
};

static struct fmt_blog blog = {0};

/**************************** Local Function ********************************/

static int32_t _get_bus_index(uint8_t msg_id)
{
	for(int i = 0 ; i < sizeof(_blog_bus) / sizeof(blog_bus_t) ; i++) {
		if(_blog_bus[i].msg_id == msg_id) {
			return i;
		}
	}

	return -1;
}

static int _file_write(const void* payload, uint16_t len)
{
	int bw;

	if(!blog.file_open) {
		/* no log file is opened */
		return 0;
	}

	// f_write(&blog.fid, payload, len, &bw);
    bw = write(blog.fid, payload, len);

	return bw;
}

static bool _buffer_check_full(uint32_t len_to_write)
{
    uint32_t free_space_in_sector = BLOG_BLOCK_SIZE - blog.buffer.index;

    // TODO: check if write multiple sectors at once

	/* check if buffer is full */
	if(free_space_in_sector < len_to_write) {
		if((blog.buffer.head + 1) % blog.buffer.num_sector == blog.buffer.tail) {
			return true;
		}
	}

    return false;
}

static void _buffer_putc(uint8_t ch)
{
    uint32_t free_space_in_sector = BLOG_BLOCK_SIZE - blog.buffer.index;

	if(free_space_in_sector < 1) {
		// move head point to next sector
		blog.buffer.head = (blog.buffer.head + 1) % blog.buffer.num_sector;
		blog.buffer.index = 0;
	}

	blog.buffer.data[blog.buffer.head * BLOG_BLOCK_SIZE + blog.buffer.index] = ch;
	blog.buffer.index += 1;
}

static void _buffer_write(const uint8_t* data, uint16_t len)
{
    uint32_t free_space_in_sector = BLOG_BLOCK_SIZE - blog.buffer.index;

    // TODO: add support with len larger than BLOG_BLOCK_SIZE

	if(free_space_in_sector < len) {
		memcpy(&blog.buffer.data[blog.buffer.head * BLOG_BLOCK_SIZE + blog.buffer.index], data, free_space_in_sector);

		// move head point to next sector
		blog.buffer.head = (blog.buffer.head + 1) % blog.buffer.num_sector;
		blog.buffer.index = 0;

		memcpy(&blog.buffer.data[blog.buffer.head * BLOG_BLOCK_SIZE + blog.buffer.index], &data[free_space_in_sector], len - free_space_in_sector);
		blog.buffer.index += len - free_space_in_sector;

		/* we have a new sector data, send blog update event to wakeup logger thread */
		logger_send_event(EVENT_BLOG_UPDATE);
	} else {
		memcpy(&blog.buffer.data[blog.buffer.head * BLOG_BLOCK_SIZE + blog.buffer.index], data, len);
		blog.buffer.index += len;
	}
}

/**************************** Public Function ********************************/

fmt_err blog_push_data(const void* payload, uint16_t len)
{
	/* chceck log status */
	if(blog.log_status != BLOG_STATUS_LOGGING) {
		return FMT_EEMPTY;
	}

    /* check if buffer has enough space to store data */
    if(_buffer_check_full(len)){
        TIMETAG_CHECK_EXECUTE(blog_buff_full1, 500, ulog_w(TAG, "buffer is full");)
        return FMT_EFULL;
    }

    /* write payload */
    _buffer_write(payload, len);

	return FMT_EOK;
}

fmt_err blog_push_msg(const uint8_t* payload, uint8_t msg_id, uint16_t len)
{
/*                           BLOG MSG Format                                 */
/*   ======================================================================= */
/*   | BLOG_BEGIN_MSG1 | BLOG_BEGIN_MSG2 | MSG_ID | PAYLOAD | BLOG_END_MSG | */ 
/*   ======================================================================= */

	int32_t bus_index;

	/* check log status */
	if(blog.log_status != BLOG_STATUS_LOGGING) {
		return FMT_EEMPTY;
	}

    /* check if buffer has enough space to store msg */
    if(_buffer_check_full(len + 4)){
        TIMETAG_CHECK_EXECUTE(blog_buff_full2, 500, ulog_w(TAG, "buffer is full");)
        return FMT_EFULL;
    }

    /* write msg begin flag */
    _buffer_putc(BLOG_BEGIN_MSG1);
    _buffer_putc(BLOG_BEGIN_MSG2);

    /* write msg id */
    _buffer_putc(msg_id);

    /* write payload */
    _buffer_write(payload, len);

    /* write msg end flag */
    _buffer_putc(BLOG_END_MSG);

	bus_index = _get_bus_index(msg_id);
	if(bus_index >= 0) {
		blog.monitor[bus_index].total_msg += 1;
	}

	return FMT_EOK;
}

fmt_err blog_add_desc(char* desc)
{
    if(strlen(desc) > BLOG_DESCRIPTION_SIZE - 1){
        ulog_w(TAG, "description too long.");
        return FMT_ENOMEM;
    }

    strcpy(blog.header.description, desc);

    return FMT_EOK;
}

fmt_err blog_start(char* file_name)
{
	if(blog.log_status != BLOG_STATUS_IDLE) {
		ulog_w(TAG, "%s is logging, stop it first", blog.file_name);
		return FMT_EBUSY;
	}

	/*********************** create log file ***********************/
	// FRESULT fres = f_open(&blog.fid, file_name, FA_OPEN_ALWAYS | FA_WRITE);
    blog.fid = open(file_name, O_CREAT | O_WRONLY);

	if(blog.fid < 0) {
		ulog_e(TAG, "%s open fail", file_name);
		return FMT_ERROR;
	}

	/* set log file open flag */
	blog.file_open = 1;

    blog.header.timestamp = systime_now_ms();

	/*********************** init log buffer ***********************/
	blog.buffer.head = blog.buffer.tail = 0;
	blog.buffer.index = 0;

	/*********************** write log header ***********************/
	blog.log_status = BLOG_STATUS_WRITE_HEAD;

	/* TODO: maybe we need check if write success */

    /* write log info */
    _file_write(&blog.header.version, sizeof(blog.header.version));
    _file_write(&blog.header.timestamp, sizeof(blog.header.timestamp));
    _file_write(&blog.header.max_name_len, sizeof(blog.header.max_name_len));
    _file_write(&blog.header.max_desc_len, sizeof(blog.header.max_desc_len));
    _file_write(blog.header.description, BLOG_DESCRIPTION_SIZE);
    // clear description after it has been written
    memset(blog.header.description, 0, BLOG_DESCRIPTION_SIZE);

	/* write bus info */
	_file_write(&blog.header.num_bus, sizeof(blog.header.num_bus));

	for(int n = 0 ; n < blog.header.num_bus ; n++) {
		_file_write(blog.header.bus_list[n].name, BLOG_MAX_NAME_LEN);
		_file_write(&blog.header.bus_list[n].msg_id, sizeof(blog.header.bus_list[n].msg_id));
		_file_write(&blog.header.bus_list[n].num_elem, sizeof(blog.header.bus_list[n].num_elem));

		// write bus element
		for(int k = 0 ; k < blog.header.bus_list[n].num_elem ; k++) {
			_file_write(blog.header.bus_list[n].elem_list[k].name, BLOG_MAX_NAME_LEN);
			_file_write(&blog.header.bus_list[n].elem_list[k].type, sizeof(blog.header.bus_list[n].elem_list[k].type));
			_file_write(&blog.header.bus_list[n].elem_list[k].number, sizeof(blog.header.bus_list[n].elem_list[k].number));
		}
	}

	/*********************** write parameter info ***********************/
	_file_write(&blog.header.num_param_group, sizeof(blog.header.num_param_group));

	char name_buffer[BLOG_MAX_NAME_LEN + 1];

	for(int n = 0 ; n < blog.header.num_param_group ; n++) {
		memset(name_buffer, 0, BLOG_MAX_NAME_LEN);
		strncpy(name_buffer, blog.header.param_group_list[n].name, BLOG_MAX_NAME_LEN);

		_file_write(name_buffer, BLOG_MAX_NAME_LEN);
		_file_write(&blog.header.param_group_list[n].param_num, sizeof(blog.header.param_group_list[n].param_num));

		for(int k = 0; k < blog.header.param_group_list[n].param_num ; k++) {
			memset(name_buffer, 0, BLOG_MAX_NAME_LEN);
			strncpy(name_buffer, blog.header.param_group_list[n].content[k].name, BLOG_MAX_NAME_LEN);

			_file_write(name_buffer, BLOG_MAX_NAME_LEN);
			_file_write(&blog.header.param_group_list[n].content[k].type, sizeof(blog.header.param_group_list[n].content[k].type));

			int type = blog.header.param_group_list[n].content[k].type;

			if(type == PARAM_TYPE_INT8) {
				_file_write(&blog.header.param_group_list[n].content[k].val.i8, sizeof(int8_t));
			} else if(type == PARAM_TYPE_UINT8) {
				_file_write(&blog.header.param_group_list[n].content[k].val.u8, sizeof(uint8_t));
			} else if(type == PARAM_TYPE_INT16) {
				_file_write(&blog.header.param_group_list[n].content[k].val.i16, sizeof(int16_t));
			} else if(type == PARAM_TYPE_UINT16) {
				_file_write(&blog.header.param_group_list[n].content[k].val.u16, sizeof(uint16_t));
			} else if(type == PARAM_TYPE_INT32) {
				_file_write(&blog.header.param_group_list[n].content[k].val.i32, sizeof(int32_t));
			} else if(type == PARAM_TYPE_UINT32) {
				_file_write(&blog.header.param_group_list[n].content[k].val.u32, sizeof(uint32_t));
			} else if(type == PARAM_TYPE_FLOAT) {
				_file_write(&blog.header.param_group_list[n].content[k].val.f, sizeof(float));
			} else if(type == PARAM_TYPE_DOUBLE) {
				_file_write(&blog.header.param_group_list[n].content[k].val.lf, sizeof(double));
			} else {
				ulog_w(TAG, "unknown parameter type:%d", type);
			}

		}
	}

	/*********************** set log status ***********************/
	strncpy(blog.file_name, file_name, sizeof(blog.file_name) - 1);

	for(int i = 0 ; i < sizeof(_blog_bus) / sizeof(blog_bus_t) ; i++) {
		blog.monitor[i].total_msg = 0;
		blog.monitor[i].lost_msg = 0;
	}

	/* start logging, set flag */
	blog.log_status = BLOG_STATUS_LOGGING;

	ulog_i(TAG, "start log:%s", file_name);

	return FMT_EOK;
}

void blog_stop(void)
{
	/* set log status to stopping, let logger thread write reamined data in
		buffer, then stop logging. */
	blog.log_status = BLOG_STATUS_STOPPING;
}

uint8_t blog_get_status(void)
{
	return blog.log_status;
}

void blog_async_output(void)
{
	uint32_t head_p, tail_p;

	if(!blog.file_open) {
		/* no log file is opened */
		return ;
	}

	OS_ENTER_CRITICAL;
	head_p = blog.buffer.head;
	tail_p = blog.buffer.tail;
	OS_EXIT_CRITICAL;

	/* write log buffer sector into storage device */
	while(head_p != tail_p) {
		// f_write(&blog.fid, &blog.buffer.data[tail_p * BLOG_BLOCK_SIZE], BLOG_BLOCK_SIZE, &bw);
        write(blog.fid, &blog.buffer.data[tail_p * BLOG_BLOCK_SIZE], BLOG_BLOCK_SIZE);

		tail_p = (tail_p + 1) % blog.buffer.num_sector;
		OS_ENTER_CRITICAL;
		blog.buffer.tail = tail_p;
		OS_EXIT_CRITICAL;

		/* sync to ensure data is actually written into the storage device,
		this is necessary if board poweroff accidently. */
		// f_sync(&blog.fid);
        fsync(blog.fid);
	}

	/* if logging is off, we need to clean up buffer. */
	if(blog.log_status == BLOG_STATUS_STOPPING) {
		// FRESULT fres = FR_OK;

		/* write rest data in buffer */
		if(blog.buffer.index) {
			// fres |= f_write(&blog.fid, &blog.buffer.data[tail_p * BLOG_BLOCK_SIZE], blog.buffer.index, &bw);
            write(blog.fid, &blog.buffer.data[tail_p * BLOG_BLOCK_SIZE], blog.buffer.index);
			// fres |= f_sync(&blog.fid);
            fsync(blog.fid);
		}

		if(blog.file_open) {
			// fres |= f_close(&blog.fid);
            close(blog.fid);
            blog.fid = -1;
			blog.file_open = 0;
		}

        ulog_i(TAG, "stop log:%s\n", blog.file_name);
        // for(int i = 0 ; i < sizeof(_blog_bus) / sizeof(blog_bus_t) ; i++) {
        // 	console_printf("%-20s msg id:%-3d total msg:%-10d lost msg:%-5d\n", _blog_bus[i].name, _blog_bus[i].msg_id,
        // 	              blog.monitor[i].total_msg, blog.monitor[i].lost_msg);
        // }
        /* set log status to idle */
        blog.log_status = BLOG_STATUS_IDLE;
	}
}

void blog_init(void)
{
	blog.file_open = 0;
	blog.log_status = BLOG_STATUS_IDLE;

    /* initialize log header */
    blog.header.version = BLOG_VERSION;
    blog.header.timestamp = 0;
    blog.header.max_name_len = BLOG_MAX_NAME_LEN;
    blog.header.max_desc_len = BLOG_DESCRIPTION_SIZE;
    memset(blog.header.description, 0, BLOG_DESCRIPTION_SIZE);

	blog.header.num_bus = sizeof(_blog_bus) / sizeof(blog_bus_t);
	blog.header.bus_list = _blog_bus;
	blog.header.num_param_group = sizeof(param_list) / sizeof(param_group_t);
	blog.header.param_group_list = (param_group_t*)&param_list;

    /* initialize log buffer */
	blog.buffer.num_sector = BLOG_BUFFER_SIZE / BLOG_BLOCK_SIZE;
	blog.buffer.head = 0;
	blog.buffer.tail = 0;
	blog.buffer.index = 0;
}