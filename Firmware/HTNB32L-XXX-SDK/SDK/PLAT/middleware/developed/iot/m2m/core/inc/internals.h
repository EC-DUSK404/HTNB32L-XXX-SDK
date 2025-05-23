/******************************************************************************

  @file    internals.h

  ---------------------------------------------------------------------------
  Copyright (c) 2016-2020 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
  ---------------------------------------------------------------------------

******************************************************************************/

/*******************************************************************************
 *
 * Copyright (c) 2013, 2014 Intel Corporation and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * The Eclipse Distribution License is available at
 *    http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    David Navarro, Intel Corporation - initial API and implementation
 *    Fabien Fleutot - Please refer to git log
 *    Toby Jaffey - Please refer to git log
 *    Bosch Software Innovations GmbH - Please refer to git log
 *    Pascal Rieux - Please refer to git log
 *    
 *******************************************************************************/
/*
 Copyright (c) 2013, 2014 Intel Corporation

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 THE POSSIBILITY OF SUCH DAMAGE.

 David Navarro <david.navarro@intel.com>

*/

#ifndef _LWM2M_INTERNALS_H_
#define _LWM2M_INTERNALS_H_

#include "liblwm2m.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "qapi_coap.h"
#include "log_utils.h"

// Macro to print device connection state
#define STR_DEVICE_CONN_STATE(S)                                                                   \
((S) == LWM2M_CLIENT_STATE_DEVICE_NOT_CONNECTED ? "LWM2M_CLIENT_STATE_DEVICE_NOT_CONNECTED" :      \
((S) == LWM2M_CLIENT_STATE_DEVICE_DM_NOT_CONNECTED ? "LWM2M_CLIENT_STATE_DEVICE_DM_NOT_CONNECTED": \
((S) == LWM2M_CLIENT_STATE_DEVICE_DM_CONNECTED ? "LWM2M_CLIENT_STATE_DEVICE_DM_CONNECTED" :        \
((S) == LWM2M_CLIENT_STATE_DEVICE_M2M_OPERATIONAL ? "LWM2M_CLIENT_STATE_DEVICE_M2M_OPERATIONAL" :  \
"Unknown"))))
// Macro to print uri
#define PRINT_URI(uriP)       \
{                             \
    if(NULL != uriP)          \
    {                         \
        if (LWM2M_URI_IS_SET_INSTANCE(uriP))              \
	{                                                     \
            if (LWM2M_URI_IS_SET_RESOURCE(uriP))          \
            {                                             \
                LOG_INFO("/%d/%d/%d",uriP->objectId, uriP->instanceId, uriP->resourceId); \
	    }                                                 \
	    else                                              \
	    {                                                 \
                LOG_INFO("/%d/%d",uriP->objectId, uriP->instanceId); \
	    }                                                 \
	}                                                     \
	else                                                  \
	{                                                     \
            LOG_INFO("/%d",uriP->objectId);             \
	}                                                     \
    }                                                     \
}
#define STR_STATE(S)                                \
((S) == STATE_INITIAL ? "STATE_INITIAL" :      \
((S) == STATE_BOOTSTRAP_REQUIRED ? "STATE_BOOTSTRAP_REQUIRED" :      \
((S) == STATE_BOOTSTRAPPING ? "STATE_BOOTSTRAPPING" :  \
((S) == STATE_REGISTER_REQUIRED ? "STATE_REGISTER_REQUIRED" :        \
((S) == STATE_REGISTERING ? "STATE_REGISTERING" :      \
((S) == STATE_READY ? "STATE_READY" :      \
"Unknown"))))))
#define STR_STATUS(S)                                           \
((S) == STATE_DEREGISTERED ? "STATE_DEREGISTERED" :             \
((S) == STATE_REG_PENDING ? "STATE_REG_PENDING" :               \
((S) == STATE_REGISTERED ? "STATE_REGISTERED" :                 \
((S) == STATE_REG_FAILED ? "STATE_REG_FAILED" :                 \
((S) == STATE_REG_UPDATE_PENDING ? "STATE_REG_UPDATE_PENDING" : \
((S) == STATE_DEREG_PENDING ? "STATE_DEREG_PENDING" :           \
((S) == STATE_BS_HOLD_OFF ? "STATE_BS_HOLD_OFF" :               \
((S) == STATE_BS_INITIATED ? "STATE_BS_INITIATED" :             \
((S) == STATE_BS_PENDING ? "STATE_BS_PENDING" :                 \
((S) == STATE_BS_FINISHED ? "STATE_BS_FINISHED" :               \
((S) == STATE_BS_FAILED ? "STATE_BS_FAILED" :                   \
((S) == STATE_BS_CONN_RETRYING ? "STATE_BS_CONN_RETRYING" :   \
"Unknown"))))))))))))
#ifdef LWM2M_WITH_LOGS
#include <inttypes.h>
#define LOG(STR) lwm2m_printf("[%s:%d] " STR "", __func__ , __LINE__)
#define LOG_ARG(FMT, ...) lwm2m_printf("[%s:%d] " FMT "", __func__ , __LINE__ , __VA_ARGS__)
#define LOG_URI(URI)                                                                \
{                                                                                   \
    if ((URI) == NULL) lwm2m_printf("[%s:%d] NULL", __func__ , __LINE__);       \
    else                                                                            \
    {                                                                               \
        lwm2m_printf("[%s:%d] /%d", __func__ , __LINE__ , (URI)->objectId);         \
        if (LWM2M_URI_IS_SET_INSTANCE(URI)) lwm2m_printf("/%d", (URI)->instanceId); \
        if (LWM2M_URI_IS_SET_RESOURCE(URI)) lwm2m_printf("/%d", (URI)->resourceId); \
        lwm2m_printf("");                                                       \
    }                                                                               \
}
#define STR_STATUS(S)                                           \
((S) == STATE_DEREGISTERED ? "STATE_DEREGISTERED" :             \
((S) == STATE_REG_PENDING ? "STATE_REG_PENDING" :               \
((S) == STATE_REGISTERED ? "STATE_REGISTERED" :                 \
((S) == STATE_REG_FAILED ? "STATE_REG_FAILED" :                 \
((S) == STATE_REG_UPDATE_PENDING ? "STATE_REG_UPDATE_PENDING" : \
((S) == STATE_DEREG_PENDING ? "STATE_DEREG_PENDING" :           \
((S) == STATE_BS_HOLD_OFF ? "STATE_BS_HOLD_OFF" :               \
((S) == STATE_BS_INITIATED ? "STATE_BS_INITIATED" :             \
((S) == STATE_BS_PENDING ? "STATE_BS_PENDING" :                 \
((S) == STATE_BS_FINISHED ? "STATE_BS_FINISHED" :               \
((S) == STATE_BS_FAILED ? "STATE_BS_FAILED" :                   \
"Unknown")))))))))))
#define STR_MEDIA_TYPE(M)                                \
((M) == LWM2M_CONTENT_TEXT ? "LWM2M_CONTENT_TEXT" :      \
((M) == LWM2M_CONTENT_LINK ? "LWM2M_CONTENT_LINK" :      \
((M) == LWM2M_CONTENT_OPAQUE ? "LWM2M_CONTENT_OPAQUE" :  \
((M) == LWM2M_CONTENT_TLV ? "LWM2M_CONTENT_TLV" :        \
((M) == LWM2M_CONTENT_JSON ? "LWM2M_CONTENT_JSON" :      \
"Unknown")))))
#else
#define LOG_ARG(FMT, ...)
#define LOG(STR)
#define LOG_URI(URI)
#endif

#define LWM2M_DEFAULT_LIFETIME  86400

#ifdef LWM2M_SUPPORT_JSON
#define REG_LWM2M_RESOURCE_TYPE     ">;rt=\"oma.lwm2m\";ct=1543,"   // Temporary value
#define REG_LWM2M_RESOURCE_TYPE_LEN 25
#else
#define REG_LWM2M_RESOURCE_TYPE     ">;rt=\"oma.lwm2m\","
#define REG_LWM2M_RESOURCE_TYPE_LEN 17
#endif
#define REG_START           "<"
#define REG_START_SIZE       1

#define REG_DEFAULT_PATH    "/"
#define REG_DEFAULT_PATH_SIZE    1


#define REG_OBJECT_MIN_LEN  5   // "</n>,"
#define REG_PATH_END        ">,"
#define REG_PATH_END_SIZE   2
#define REG_PATH_SEPARATOR  "/"
#define REG_PATH_SEPARATOR_SIZE  1


#define REG_OBJECT_PATH             "<%s/%hu>,"
#define REG_OBJECT_INSTANCE_PATH    "<%s/%hu/%hu>,"

#define LWM2M_ENABLER_VERSION_1_0    "</>;lwm2m=1.0,"
#define LWM2M_ENABLER_VERSION_1_1    "</>;lwm2m=1.1,"
#define LWM2M_ENABLER_VERSION_SIZE    14


#define URI_REGISTRATION_SEGMENT        "rd"
#define URI_REGISTRATION_SEGMENT_LEN    2
#define URI_BOOTSTRAP_SEGMENT           "bs"
#define URI_BOOTSTRAP_SEGMENT_LEN       2

#define QUERY_SMS             "sms="
#define QUERY_SMS_LEN         4
#define QUERY_LIFETIME        "lt="
#define QUERY_LIFETIME_LEN    3
#define QUERY_VERSION         "lwm2m="
#define QUERY_VERSION_1_0     "1.0"
#define QUERY_VERSION_1_1     "1.1"
#define QUERY_VERSION_LEN     6
#define QUERY_BINDING         "b="
#define QUERY_BINDING_LEN     2
#define QUERY_QUEUE_MODE      "q="
#define QUERY_QUEUE_MODE_LEN  2
#define QUERY_DELIMITER       "&"

#define QUERY_VERSION_FULL      "lwm2m=1.0"
#define BS_DISC_PATH_END        ">"
#define BS_DISC_PATH_END_SIZE    1

#define BS_DISC_OBJ_PATH_SEPARATOR  "/"
#define BS_DISC_SSID        ";ssid="
#define BS_DISC_SSID_SIZE      6
#define BS_DISC_URI_START   ";uri=\""
#define BS_DISC_URI_START_SIZE   6

#define BS_DISC_URI_END     "\""
#define BS_DISC_URI_END_SIZE   1



#define BS_DISC_COMMA ","
#define BS_DISC_COMMA_SIZE  1


#define QUERY_VERSION_FULL_LEN  9

#define REG_URI_START       '<'
#define REG_URI_END         '>'
#define REG_DELIMITER       ','
#define REG_ATTR_SEPARATOR  ';'
#define REG_ATTR_EQUALS     '='
#define REG_ATTR_TYPE_KEY           "rt"
#define REG_ATTR_TYPE_KEY_LEN       2
#define REG_ATTR_TYPE_VALUE         "\"oma.lwm2m\""
#define REG_ATTR_TYPE_VALUE_LEN     11
#define REG_ATTR_CONTENT_KEY        "ct"
#define REG_ATTR_CONTENT_KEY_LEN    2
#define REG_ATTR_CONTENT_JSON       "1543"   // Temporary value
#define REG_ATTR_CONTENT_JSON_LEN   4

#define ATTR_SERVER_ID_STR       "ep="
#define ATTR_SERVER_ID_LEN       3
#define ATTR_MIN_PERIOD_STR      "pmin="
#define ATTR_MIN_PERIOD_LEN      5
#define ATTR_MAX_PERIOD_STR      "pmax="
#define ATTR_MAX_PERIOD_LEN      5
#define ATTR_GREATER_THAN_STR    "gt="
#define ATTR_GREATER_THAN_LEN    3
#define ATTR_LESS_THAN_STR       "lt="
#define ATTR_LESS_THAN_LEN       3
#define ATTR_STEP_STR            "st="
#define ATTR_STEP_LEN            3
#define ATTR_DIMENSION_STR       "dim="
#define ATTR_DIMENSION_LEN       4

#define URI_MAX_STRING_LEN    26      // /65535/65535/65535/65535
#define _PRV_64BIT_BUFFER_SIZE 8

#define LINK_ITEM_START             "<"
#define LINK_ITEM_START_SIZE        1
#define LINK_ITEM_END               ">,"
#define LINK_ITEM_END_SIZE          2
#define LINK_ITEM_DIM_START         ">;dim="
#define LINK_ITEM_DIM_START_SIZE    6
#define LINK_ITEM_ATTR_END          ","
#define LINK_ITEM_ATTR_END_SIZE     1
#define LINK_URI_SEPARATOR          "/"
#define LINK_URI_SEPARATOR_SIZE     1
#define LINK_ATTR_SEPARATOR         ";"
#define LINK_ATTR_SEPARATOR_SIZE    1

#define ATTR_FLAG_NUMERIC (uint8_t)(LWM2M_ATTR_FLAG_LESS_THAN | LWM2M_ATTR_FLAG_GREATER_THAN | LWM2M_ATTR_FLAG_STEP)

#define LWM2M_URI_FLAG_DM           (uint8_t)0x00
#define LWM2M_URI_FLAG_ALL_OBJECTS  (uint8_t)0x10
#define LWM2M_URI_FLAG_REGISTRATION (uint8_t)0x20
#define LWM2M_URI_FLAG_BOOTSTRAP    (uint8_t)0x40

#define LWM2M_URI_MASK_TYPE (uint8_t)0x70
#define LWM2M_URI_MASK_ID   (uint8_t)0x07

#define CHECK_SHORT_SERVER_ID_RANGE(ID) (0 < (ID) && (ID) < LWM2M_MAX_ID)
typedef struct
{
  lwm2m_uri_t uri;
  lwm2m_result_callback_t callback;
  void * userData;
} dm_data_t;

typedef enum
{
  URI_DEPTH_OBJECT,
  URI_DEPTH_OBJECT_INSTANCE,
  URI_DEPTH_RESOURCE,
  URI_DEPTH_RESOURCE_INSTANCE
} uri_depth_t;

typedef struct 
{
  uint8_t token[QAPI_COAP_TOKEN_LEN];
  uint16_t  mid;
  uint16_t  shortID;
  uint8_t token_len;
} lwm2m_response_info_t;

// defined in uri.c
lwm2m_uri_t * uri_decode(char * altPath, qapi_Multi_Option_t *uriPath);
int uri_getNumber(uint8_t * uriString, size_t uriLength);
int uri_toString(lwm2m_uri_t * uriP, uint8_t * buffer, size_t bufferLen, uri_depth_t * depthP);
int prv_parseNumber(uint8_t * uriString,
    size_t uriLength,
    size_t * headP);

// defined in objects.c
bool object_isObjOrObjInstanceExist(lwm2m_context_t * contextP, lwm2m_uri_t * uriP);

qapi_Coap_Status_t checkAuthForRead(lwm2m_context_t * contextP,
    lwm2m_uri_t * uriP);

uint8_t object_readData(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                        int * sizeP, lwm2m_data_t ** dataP, lwm2m_server_t *serverP);

uint8_t object_read(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                    lwm2m_media_type_t * formatP, uint8_t ** bufferP, 
                    size_t * lengthP, lwm2m_data_t **dataP, int *sizeP, lwm2m_server_t *serverP);

uint8_t object_write(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                     lwm2m_media_type_t format, qapi_Coap_Packet_t* buffer,
                     size_t length, uint8_t write_method, bool bs_true);
uint8_t object_create(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                      lwm2m_media_type_t format, uint8_t * buffer, 
                      size_t length);

uint8_t object_execute(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                       uint8_t * buffer, size_t length);

uint8_t object_delete(lwm2m_context_t * contextP, lwm2m_uri_t * uriP);

uint8_t object_delete_bootstrap(lwm2m_context_t * contextP, lwm2m_uri_t * uriP);

uint8_t object_discover(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                        uint8_t ** bufferP, size_t * lengthP, lwm2m_server_t *serverP);
uint8_t object_bs_discover(lwm2m_context_t * contextP, uint16_t objectId,
                        uint8_t ** bufferP,  size_t * bufferLen);

uint8_t validate_ex_object_permissions(lwm2m_context_t *contextP,
                        lwm2m_uri_t * uriP, qapi_Net_LWM2M_DL_Msg_t msg_type);


uint8_t object_checkReadable(lwm2m_context_t * contextP, lwm2m_uri_t * uriP, lwm2m_server_t * serverP);
uint8_t object_checkNumeric(lwm2m_context_t * contextP, lwm2m_uri_t * uriP, lwm2m_server_t * serverP);

bool object_isInstanceNew(lwm2m_context_t * contextP, uint16_t objectId,
                          uint16_t instanceId);
int object_getRegisterPayload(lwm2m_context_t * contextP, lwm2m_server_t * server, uint8_t * buffer, size_t length);
int get_registerPayload_size(lwm2m_context_t * contextP);

int object_getServers(lwm2m_context_t * contextP);
int object_getBSServer(lwm2m_context_t * contextP);

uint8_t object_createInstance(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                              lwm2m_data_t * dataP);

uint8_t object_writeInstance(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                             lwm2m_data_t * dataP, uint8_t write_method);

int object_UpdateServerList(lwm2m_context_t * contextP, 
    uint16_t short_serverid);

bool object_checkACLConsistence(lwm2m_context_t * contextP, bool isClientInitBS);

bool object_checkSecurityObjInstConsistence(lwm2m_context_t * contextP);

bool object_checkServerObjInstConsistence(lwm2m_context_t * contextP);

bool object_checkServerListConsistence(lwm2m_context_t * contextP);

bool object_isSSIDUnique(lwm2m_context_t * contextP);

// defined in management.c
uint8_t dm_handleRequest(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                         lwm2m_server_t * serverP, qapi_Coap_Packet_t * message,
                         qapi_Coap_Packet_t * response);

uint8_t dm_handleRequest_ExObj(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                         lwm2m_server_t * serverP, qapi_Coap_Packet_t * message,
                         qapi_Coap_Packet_t * response);


// defined in observe.c
uint32_t get_pmin_value(lwm2m_context_t * contextP, lwm2m_watcher_t * watcherP,
                               lwm2m_uri_t uri);
uint32_t get_pmax_value(lwm2m_context_t * contextP, lwm2m_watcher_t * watcherP,
                               lwm2m_uri_t uri);

uint8_t observe_handleRequest(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                              lwm2m_server_t * serverP, int size,
                              lwm2m_data_t * dataP, qapi_Coap_Packet_t * message,
                              qapi_Coap_Packet_t * response);

void observe_delete(lwm2m_context_t *contextP,
                    lwm2m_uri_t *uriP);

void observe_cancel(lwm2m_context_t * contextP, uint16_t mid, void * fromSessionH, lwm2m_uri_t * uriP);
uint8_t observe_setParameters(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                              lwm2m_server_t * serverP, lwm2m_attributes_t * attrP);

void observe_step(lwm2m_context_t * contextP, time_t currentTime, time_t * timeoutP);

bool observe_handleNotify(lwm2m_context_t * contextP, void * fromSessionH,
                          qapi_Coap_Packet_t * message, qapi_Coap_Packet_t * response);
void observe_remove(lwm2m_client_t * clientP, lwm2m_observation_t * observationP);
lwm2m_observed_t * observe_findByUri(lwm2m_context_t * contextP, lwm2m_uri_t * uriP);

// defined in registration.c
uint8_t registration_handleRequest(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                                   void * fromSessionH, qapi_Coap_Packet_t * message,
                                   qapi_Coap_Packet_t * response);
void registration_deregister(lwm2m_context_t * contextP, lwm2m_server_t * serverP);
void registration_freeClient(lwm2m_client_t * clientP);
uint8_t registration_start(lwm2m_context_t * contextP, lwm2m_server_t * serverP, time_t * timeoutP);
int registration_step(lwm2m_context_t * contextP, time_t currentTime, time_t * timeoutP);
lwm2m_status_t registration_getStatus(lwm2m_context_t * contextP, bool *blocking);
lwm2m_status_t get_registration_update_status(lwm2m_context_t * contextP);
void prv_deregister(lwm2m_context_t * contextP);
lwm2m_status_t get_current_regstatus(lwm2m_context_t * contextP);
// defined in packet.c
int32_t lwm2m_handle_request_cb(qapi_Coap_Session_Hdl_t  hdl,
    qapi_Coap_Packet_t* messageP,
    void * clientData);

void lwm2m_Coap_Event_Callback(qapi_Coap_Session_Hdl_t hdl , qapi_Coap_Packet_t * dl_pkt , qapi_Coap_Event_t lwm2m_event , void * event_info);

// defined in bootstrap.c
void bootstrap_step(lwm2m_context_t * contextP, uint32_t currentTime,
                    time_t* timeoutP);
uint8_t bootstrap_handleCommand(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                                lwm2m_server_t * serverP, qapi_Coap_Packet_t * message,
                                qapi_Coap_Packet_t * response);
uint8_t bootstrap_handleDeleteAll(lwm2m_context_t * context, void * fromSessionH);
uint8_t bootstrap_handleFinish(lwm2m_context_t * context, void * fromSessionH);
uint8_t bootstrap_handleRequest(lwm2m_context_t * contextP, lwm2m_uri_t * uriP,
                                void * fromSessionH, qapi_Coap_Packet_t * message,
                                qapi_Coap_Packet_t * response);
void bootstrap_start(lwm2m_context_t * contextP);
lwm2m_status_t bootstrap_getStatus(lwm2m_context_t * contextP);
bool bootstrap_checkBSserverAccount(lwm2m_context_t * contextP, bool isClientInitBS);
bool bootstrap_checkAllConsistence(lwm2m_context_t *contextP, bool isClientInitBS);

// defined in tlv.c
int tlv_parse(uint8_t * buffer, size_t bufferLen, lwm2m_data_t ** dataP);
size_t tlv_serialize(bool isResourceInstance, int size, lwm2m_data_t * dataP,
                     uint8_t ** bufferP);

// defined in json.c
#ifdef LWM2M_SUPPORT_JSON
int json_parse(lwm2m_uri_t * uriP, uint8_t * buffer, size_t bufferLen,
               lwm2m_data_t ** dataP);
size_t json_serialize(lwm2m_uri_t * uriP, int size, lwm2m_data_t * tlvP,
    uint8_t ** bufferP);
#endif

// defined in discover.c
int discover_serialize(lwm2m_context_t * contextP, lwm2m_uri_t * uriP, int size,
                       lwm2m_data_t * dataP, uint8_t ** bufferP);

int discover_serialize_wo_inst(lwm2m_context_t * contextP,
                               lwm2m_uri_t * uriP,
                               uint8_t ** bufferP);

// defined in utils.c
lwm2m_data_type_t utils_depthToDatatype(uri_depth_t depth);
lwm2m_binding_t utils_stringToBinding(uint8_t *buffer, size_t length);
lwm2m_media_type_t utils_convertMediaType(qapi_Coap_Content_Type_t type);
int utils_isAltPathValid(const char * altPath);
int utils_stringCopy(char * buffer, size_t length, const char * str);
int utils_intCopy(char * buffer, size_t length, int32_t value);
size_t utils_intToText(int64_t data, uint8_t * string, size_t length);
size_t utils_floatToText(double data, uint8_t * string, size_t length);
int utils_plainTextToInt64(uint8_t * buffer, int length, int64_t * dataP);
int utils_plainTextToFloat64(uint8_t * buffer, int length, double * dataP);
size_t utils_int64ToPlainText(int64_t data, uint8_t ** bufferP);
size_t utils_float64ToPlainText(double data, uint8_t ** bufferP);
size_t utils_boolToPlainText(bool data, uint8_t ** bufferP);
void utils_copyValue(void * dst, const void * src, size_t len);
int utils_opaqueToInt(const uint8_t * buffer, size_t buffer_len, int64_t * dataP);
int utils_opaqueToFloat(const uint8_t * buffer, size_t buffer_len, double * dataP);
size_t utils_encodeInt(int64_t data, uint8_t data_buffer[_PRV_64BIT_BUFFER_SIZE]);
size_t utils_encodeObjectLink(int64_t data, uint8_t data_buffer[_PRV_64BIT_BUFFER_SIZE]);
size_t utils_encodeFloat(double data, uint8_t data_buffer[_PRV_64BIT_BUFFER_SIZE]);
size_t utils_base64ToOpaque(uint8_t * dataP, size_t dataLen, uint8_t ** bufferP);
size_t utils_opaqueToBase64(uint8_t * dataP, size_t dataLen, uint8_t ** bufferP);
size_t utils_base64Encode(uint8_t * dataP, size_t dataLen, uint8_t * bufferP,
                          size_t bufferLen);
size_t prv_getBase64Size(size_t dataLen);
#ifdef LWM2M_CLIENT_MODE
lwm2m_server_t * utils_findServer(lwm2m_context_t * contextP, void * fromSessionH);
lwm2m_server_t * utils_findBootstrapServer(lwm2m_context_t * contextP, 
                                           void * fromSessionH);

lwm2m_handle_info_t * retrieve_lwm2m_ctx_info_from_id(uint16_t objectId);

size_t utils_numDigit(int64_t data);


bool lwm2m_coap_sessionhandle_is_equal(void * handle1,
    void * handle2,
    void * userData);

#endif

/** defined in m2m_dss_util.c
 * These API's are specific to target platform
 */
int m2m_release_datapath(void);
int m2m_establish_datapath(void);
int m2m_get_local_ipaddr(struct in_addr* ipaddr);
char *get_localip_for_server(uint16_t shortID);

#endif
