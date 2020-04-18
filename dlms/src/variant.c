//
// --------------------------------------------------------------------------
//  Gurux Ltd
//
// --------------------------------------------------------------------------
//  Gurux Ltd
//
//
//
// Filename:        $HeadURL$
//
// Version:         $Revision$,
//                  $Date$
//                  $Author$
//
// Copyright (c) Gurux Ltd
//
//---------------------------------------------------------------------------
//
//  DESCRIPTION
//
// This file is a part of Gurux Device Framework.
//
// Gurux Device Framework is Open Source software; you can redistribute it
// and/or modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; version 2 of the License.
// Gurux Device Framework is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// This code is licensed under the GNU General Public License v2.
// Full text may be retrieved at http://www.gnu.org/licenses/gpl-2.0.txt
//---------------------------------------------------------------------------

#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
#include <assert.h>
#endif

#include "../include/gxmem.h"
#ifndef GX_DLMS_MICROCONTROLLER
#include <stdio.h> //printf needs this or error is generated.
#if _MSC_VER > 1400
#include <crtdbg.h>
#endif
#endif //GX_DLMS_MICROCONTROLLER

#include <string.h> /* memset */

#include "../include/variant.h"
#include "../include/errorcodes.h"
#include "../include/helpers.h"

int var_setEnum(dlmsVARIANT *data, unsigned char value)
{
    var_clear(data);
    data->vt = DLMS_DATA_TYPE_ENUM;
    data->bVal = value;
    return DLMS_ERROR_CODE_OK;
}

int var_setUInt8(dlmsVARIANT *data, unsigned char value)
{
    var_clear(data);
    data->vt = DLMS_DATA_TYPE_UINT8;
    data->bVal = value;
    return DLMS_ERROR_CODE_OK;
}

int var_setUInt16(dlmsVARIANT *data, unsigned short value)
{
    var_clear(data);
    data->vt = DLMS_DATA_TYPE_UINT16;
    data->uiVal = value;
    return DLMS_ERROR_CODE_OK;
}

int var_setUInt32(dlmsVARIANT *data, unsigned long value)
{
    var_clear(data);
    data->vt = DLMS_DATA_TYPE_UINT32;
    data->lVal = value;
    return DLMS_ERROR_CODE_OK;
}

int var_setUInt64(dlmsVARIANT *data, unsigned long long value)
{
    var_clear(data);
    data->vt = DLMS_DATA_TYPE_UINT64;
    data->ullVal = value;
    return DLMS_ERROR_CODE_OK;
}

int var_setInt8(dlmsVARIANT *data, char value)
{
    var_clear(data);
    data->vt = DLMS_DATA_TYPE_INT8;
    data->cVal = value;
    return DLMS_ERROR_CODE_OK;
}

int var_setInt16(dlmsVARIANT *data, short value)
{
    var_clear(data);
    data->vt = DLMS_DATA_TYPE_INT16;
    data->iVal = value;
    return DLMS_ERROR_CODE_OK;
}

int var_setInt32(dlmsVARIANT *data, long value)
{
    var_clear(data);
    data->vt = DLMS_DATA_TYPE_INT32;
    data->lVal = value;
    return DLMS_ERROR_CODE_OK;
}

int var_setInt64(dlmsVARIANT *data, long long value)
{
    var_clear(data);
    data->vt = DLMS_DATA_TYPE_INT64;
    data->llVal = value;
    return DLMS_ERROR_CODE_OK;
}

int var_setDouble(dlmsVARIANT *data, double value)
{
    var_clear(data);
    data->vt = DLMS_DATA_TYPE_FLOAT64;
    data->dblVal = value;
    return DLMS_ERROR_CODE_OK;
}

int var_setFloat(dlmsVARIANT *data, float value)
{
    var_clear(data);
    data->vt = DLMS_DATA_TYPE_FLOAT32;
    data->fltVal = value;
    return DLMS_ERROR_CODE_OK;
}


int var_getUInt8(dlmsVARIANT *data, unsigned char* value)
{
    if (data->vt == DLMS_DATA_TYPE_NONE)
    {
        *value = 0;
    }
    else
    {
        *value = data->bVal;
    }
    return DLMS_ERROR_CODE_OK;
}

int var_getUInt16(dlmsVARIANT *data, unsigned short* value)
{
    if (data->vt == DLMS_DATA_TYPE_NONE)
    {
        *value = 0;
    }
    else
    {
        *value = data->uiVal;
    }
    return DLMS_ERROR_CODE_OK;
}

int var_getUInt32(dlmsVARIANT *data, unsigned long* value)
{
    if (data->vt == DLMS_DATA_TYPE_NONE)
    {
        *value = 0;
    }
    else
    {
        *value = data->lVal;
    }
    return DLMS_ERROR_CODE_OK;
}

int var_getUInt64(dlmsVARIANT *data, unsigned long long* value)
{
    if (data->vt == DLMS_DATA_TYPE_NONE)
    {
        *value = 0;
    }
    else
    {
        *value = data->ullVal;
    }
    return DLMS_ERROR_CODE_OK;
}

int var_getInt8(dlmsVARIANT *data, char* value)
{
    if (data->vt == DLMS_DATA_TYPE_NONE)
    {
        *value = 0;
    }
    else
    {
        *value = data->cVal;
    }
    return DLMS_ERROR_CODE_OK;
}

int var_getInt16(dlmsVARIANT *data, short* value)
{
    if (data->vt == DLMS_DATA_TYPE_NONE)
    {
        *value = 0;
    }
    else
    {
        *value = data->iVal;
    }
    return DLMS_ERROR_CODE_OK;
}

int var_getInt32(dlmsVARIANT *data, long* value)
{
    var_clear(data);
    if (data->vt == DLMS_DATA_TYPE_NONE)
    {
        *value = 0;
    }
    else
    {
        *value = data->lVal;
    }
    return DLMS_ERROR_CODE_OK;
}

int var_getInt64(dlmsVARIANT *data, long long* value)
{
    if (data->vt == DLMS_DATA_TYPE_NONE)
    {
        *value = 0;
    }
    else
    {
        *value = data->llVal;
    }
    return DLMS_ERROR_CODE_OK;
}

int var_addBytes(dlmsVARIANT *data, const unsigned char* value, unsigned short count)
{
    if (count < 0)
    {
        return DLMS_ERROR_CODE_INVALID_PARAMETER;
    }
    if (data->vt != DLMS_DATA_TYPE_OCTET_STRING)
    {
        var_clear(data);
        data->byteArr = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
        bb_init(data->byteArr);
        data->vt = DLMS_DATA_TYPE_OCTET_STRING;
    }
    else
    {
        bb_clear(data->byteArr);
    }
    return bb_set(data->byteArr, value, count);
}

int var_setString(dlmsVARIANT *data, const char* value, unsigned short count)
{
    var_clear(data);
    if (count < 0)
    {
        return DLMS_ERROR_CODE_INVALID_PARAMETER;
    }
    if (data->vt != DLMS_DATA_TYPE_STRING)
    {
        var_clear(data);
        data->strVal = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
        bb_init(data->strVal);
        data->vt = DLMS_DATA_TYPE_STRING;
    }
    bb_set(data->strVal, (const unsigned char*)value, count);
    return DLMS_ERROR_CODE_OK;
}

int var_addOctetString(
    dlmsVARIANT *data,
    gxByteBuffer* ba)
{
    var_clear(data);
    return var_addBytes(data, ba->data + ba->position, (unsigned short)(ba->size - ba->position));
}

int var_addByteArray(
    dlmsVARIANT *data,
    gxByteBuffer* ba,
    unsigned short index,
    unsigned short count)
{
    return var_addBytes(data, ba->data + index, count);
}

//Initialize variant.
int var_init(dlmsVARIANT *data)
{
    data->vt = DLMS_DATA_TYPE_NONE;
    data->byteArr = NULL;
    return DLMS_ERROR_CODE_OK;
}

void var_attachArray(dlmsVARIANT * data,
    const dlmsVARIANT** arr,
    const unsigned short count)
{
    data->Arr = (variantArray*)gxmalloc(sizeof(variantArray));
    data->vt = DLMS_DATA_TYPE_ARRAY;
    data->Arr->capacity = 0x8000 + count;
    data->Arr->size = count;
    data->Arr->position = 0;
    data->Arr->data = (void**)arr;
}

void var_attachStructure(dlmsVARIANT * data,
    const dlmsVARIANT** arr,
    const unsigned short count)
{
    data->Arr = (variantArray*)gxmalloc(sizeof(variantArray));
    data->vt = DLMS_DATA_TYPE_STRUCTURE;
    data->Arr->capacity = 0x8000 + count;
    data->Arr->size = count;
    data->Arr->position = 0;
    data->Arr->data = (void**)arr;
}

//Clear variant.
int var_clear(dlmsVARIANT *data)
{
    if (data->vt == DLMS_DATA_TYPE_OCTET_STRING ||
        data->vt == DLMS_DATA_TYPE_COMPACT_ARRAY)
    {
        if (data->byteArr != NULL)
        {
            bb_clear(data->byteArr);
            if (!bb_isAttached(data->byteArr))
            {
                gxfree(data->byteArr);
            }
            data->byteArr = NULL;
        }
    }
    else if (data->vt == DLMS_DATA_TYPE_STRING)
    {
        if (data->strVal != NULL)
        {
            bb_clear(data->strVal);
            gxfree(data->strVal);
        }
    }
    else if (data->vt == DLMS_DATA_TYPE_ARRAY ||
        data->vt == DLMS_DATA_TYPE_STRUCTURE)
    {
        if (data->Arr != NULL)
        {
            va_clear(data->Arr);
            gxfree(data->Arr);
        }
    }
    else if (data->vt == DLMS_DATA_TYPE_BIT_STRING)
    {
        if (data->bitArr != NULL)
        {
            ba_clear(data->bitArr);
            gxfree(data->bitArr);
        }
    }
    else if (data->vt == DLMS_DATA_TYPE_DATETIME ||
        data->vt == DLMS_DATA_TYPE_DATE ||
        data->vt == DLMS_DATA_TYPE_TIME)
    {
        if (data->dateTime != NULL)
        {
            gxfree(data->dateTime);
            data->dateTime = NULL;
        }
    }
    data->llVal = 0;
    data->vt = DLMS_DATA_TYPE_NONE;
    return DLMS_ERROR_CODE_OK;
}

int var_getDateTime2(
    gxtime *dateTime,
    gxByteBuffer* ba)
{
    unsigned short year = 0xFFFF;
    //Add year.
    if (dateTime->value.tm_year != -1 && (dateTime->skip & DATETIME_SKIPS_YEAR) == 0)
    {
        year = (unsigned short)(1900 + dateTime->value.tm_year);
    }
    bb_setUInt16(ba, year);
    //Add month
    if (dateTime->daylightSavingsBegin)
    {
        bb_setUInt8(ba, 0xFE);
    }
    else if (dateTime->daylightSavingsEnd)
    {
        bb_setUInt8(ba, 0xFD);
    }
    else if (dateTime->value.tm_mon != -1 && (dateTime->skip & DATETIME_SKIPS_MONTH) == 0)
    {
        bb_setUInt8(ba, (unsigned char)(dateTime->value.tm_mon + 1));
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
    //Add day
    if (dateTime->value.tm_mday != -1 && (dateTime->skip & DATETIME_SKIPS_DAY) == 0)
    {
        bb_setUInt8(ba, (unsigned char)dateTime->value.tm_mday);
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
    //Add week day
    if (dateTime->value.tm_wday != -1 && (dateTime->skip & DATETIME_SKIPS_DAYOFWEEK) == 0)
    {
        unsigned char val = (unsigned char)dateTime->value.tm_wday;
        //If Sunday.
        if (val == 0)
        {
            val = 7;
        }
        bb_setUInt8(ba, val);
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }

    //Add Hours
    if (dateTime->value.tm_hour != -1 && (dateTime->skip & DATETIME_SKIPS_HOUR) == 0)
    {
        bb_setUInt8(ba, (unsigned char)dateTime->value.tm_hour);
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
    //Add Minutes
    if (dateTime->value.tm_min != -1 && (dateTime->skip & DATETIME_SKIPS_MINUTE) == 0)
    {
        bb_setUInt8(ba, (unsigned char)dateTime->value.tm_min);
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
    //Add seconds.
    if (dateTime->value.tm_sec != -1 && (dateTime->skip & DATETIME_SKIPS_SECOND) == 0)
    {
        bb_setUInt8(ba, (unsigned char)dateTime->value.tm_sec);
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
    //Add ms.
#ifdef DLMS_ITALIAN_STANDARD
    //Italian standard uses 0 for ms.
    bb_setUInt8(ba, 0x00);
#else
    if ((dateTime->skip & DATETIME_SKIPS_MS) == 0)
    {
        bb_setUInt8(ba, 0x00);
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
#endif //DLMS_ITALIAN_STANDARD

    //Add Deviation
    if (dateTime->value.tm_year == -1 || (dateTime->skip & DATETIME_SKIPS_DEVITATION) != 0)
    {
        bb_setInt16(ba, 0x8000);//(not specified)
    }
    else
    {
        bb_setInt16(ba, dateTime->deviation);
    }
    //Add clock status
    if (dateTime->value.tm_isdst)
    {
        bb_setUInt8(ba, dateTime->status | DLMS_CLOCK_STATUS_DAYLIGHT_SAVE_ACTIVE);
    }
    else
    {
        bb_setUInt8(ba, dateTime->status);
    }
    return 0;
}

int var_getDate(
    gxtime * date,
    gxByteBuffer* ba)
{
    unsigned short year = 0xFFFF;
    //Add year.
    if (date->value.tm_year != -1 && (date->skip & DATETIME_SKIPS_YEAR) == 0)
    {
        year = (unsigned short)(1900 + date->value.tm_year);
    }
    bb_setUInt16(ba, year);
    //Add month
    if (date->daylightSavingsBegin)
    {
        bb_setUInt8(ba, 0xFE);
    }
    else if (date->daylightSavingsEnd)
    {
        bb_setUInt8(ba, 0xFD);
    }
    else if (date->value.tm_mon != -1 && (date->skip & DATETIME_SKIPS_MONTH) == 0)
    {
        bb_setUInt8(ba, (unsigned char)(date->value.tm_mon + 1));
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
    //Add day
    if (date->value.tm_mday != -1 && (date->skip & DATETIME_SKIPS_DAY) == 0)
    {
        bb_setUInt8(ba, (unsigned char)date->value.tm_mday);
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
    //Add week day
    bb_setUInt8(ba, 0xFF);
    return 0;
}

int var_getTime(
    gxtime * date,
    gxByteBuffer* ba)
{
    //Add Hours
    if (date->value.tm_hour != -1 && (date->skip & DATETIME_SKIPS_HOUR) == 0)
    {
        bb_setUInt8(ba, (unsigned char)date->value.tm_hour);
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
    //Add Minutes
    if (date->value.tm_min != -1 && (date->skip & DATETIME_SKIPS_MINUTE) == 0)
    {
        bb_setUInt8(ba, (unsigned char)date->value.tm_min);
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
    //Add seconds.
    if (date->value.tm_sec != -1 && (date->skip & DATETIME_SKIPS_SECOND) == 0)
    {
        bb_setUInt8(ba, (unsigned char)date->value.tm_sec);
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
    //Add ms.
    if ((date->skip & DATETIME_SKIPS_MS) == 0)
    {
        bb_setUInt8(ba, 0);
    }
    else
    {
        bb_setUInt8(ba, 0xFF);
    }
    return 0;
}

//Get bytes from variant value.
int var_getBytes(
    dlmsVARIANT *data,
    gxByteBuffer* ba)
{
    return var_getBytes2(data, data->vt, ba);
}

/**
* Convert octetstring to DLMS bytes.
*
* buff
*            Byte buffer where data is write.
* value
*            Added value.
*/
int var_setOctetString(gxByteBuffer* buff, dlmsVARIANT* value)
{
    if (value->vt == DLMS_DATA_TYPE_STRING)
    {
        gxByteBuffer bb;
        bb_init(&bb);
        bb_addHexString(&bb, (char*)value->strVal->data);
        hlp_setObjectCount(bb.size, buff);
        bb_set2(buff, &bb, 0, bb.size);
    }
    else if (value->vt == DLMS_DATA_TYPE_OCTET_STRING)
    {
        if (value->byteArr == NULL)
        {
            hlp_setObjectCount(0, buff);
        }
        else
        {
            hlp_setObjectCount(value->byteArr->size, buff);
            bb_set(buff, value->byteArr->data, value->byteArr->size);
        }
    }
    else if (value->vt == DLMS_DATA_TYPE_NONE)
    {
        hlp_setObjectCount(0, buff);
    }
    else
    {
        // Invalid data type.
        return DLMS_ERROR_CODE_INVALID_PARAMETER;
    }
    return 0;
}


//Returns bytes as Big Endian byteorder.
int var_getBytes2(
    dlmsVARIANT *data,
    DLMS_DATA_TYPE type,
    gxByteBuffer* ba)
{
    int ret, pos;
    if (type == DLMS_DATA_TYPE_STRUCTURE ||
        type == DLMS_DATA_TYPE_ARRAY)
    {
        dlmsVARIANT* tmp;
        bb_setUInt8(ba, data->vt);
        hlp_setObjectCount(data->Arr->size, ba);
        for (pos = 0; pos != data->Arr->size; ++pos)
        {
            if ((ret = va_getByIndex(data->Arr, pos, &tmp)) != 0)
            {
                return ret;
            }
            ret = var_getBytes(tmp, ba);
            if (ret != DLMS_ERROR_CODE_OK)
            {
                return ret;
            }
        }
        return 0;
    }
    if ((ret = bb_setUInt8(ba, type)) != 0)
    {
        return ret;
    }
    switch (type)
    {
    case DLMS_DATA_TYPE_NONE:
        break;
    case DLMS_DATA_TYPE_UINT8:
    case DLMS_DATA_TYPE_ENUM:
        ret = bb_setUInt8(ba, data->bVal);
        break;
    case DLMS_DATA_TYPE_BOOLEAN:
        ret = bb_setUInt8(ba, data->bVal == 0 ? 0 : 1);
        break;
    case DLMS_DATA_TYPE_UINT16:
        ret = bb_setUInt16(ba, data->uiVal);
        break;
    case DLMS_DATA_TYPE_UINT32:
        ret = bb_setUInt32(ba, data->ulVal);
        break;
    case DLMS_DATA_TYPE_UINT64:
        ret = bb_setUInt64(ba, data->ullVal);
        break;
    case DLMS_DATA_TYPE_INT8:
        ret = bb_setUInt8(ba, data->cVal);
        break;
    case DLMS_DATA_TYPE_INT16:
        ret = bb_setInt16(ba, data->uiVal);
        break;
    case DLMS_DATA_TYPE_INT32:
        ret = bb_setInt32(ba, data->lVal);
        break;
    case DLMS_DATA_TYPE_INT64:
        ret = bb_setInt64(ba, data->llVal);
        break;
    case DLMS_DATA_TYPE_FLOAT32:
#ifndef GX_DLMS_MICROCONTROLLER
        ret = bb_setFloat(ba, data->fltVal);
#else
        ret = DLMS_ERROR_CODE_INVALID_PARAMETER;
#endif //GX_DLMS_MICROCONTROLLER
        break;
    case DLMS_DATA_TYPE_FLOAT64:
#ifndef GX_DLMS_MICROCONTROLLER
        ret = bb_setDouble(ba, data->dblVal);
#else
        ret = DLMS_ERROR_CODE_INVALID_PARAMETER;
#endif //GX_DLMS_MICROCONTROLLER
        break;
    case DLMS_DATA_TYPE_STRING:
        if (data->strVal == NULL)
        {
            ret = hlp_setObjectCount(0, ba);
        }
        else
        {
            if ((ret = hlp_setObjectCount(data->strVal->size, ba)) == 0)
            {
                ret = bb_set(ba, data->strVal->data, data->strVal->size);
            }
        }
        break;
    case DLMS_DATA_TYPE_OCTET_STRING:
        if (data->vt == DLMS_DATA_TYPE_DATETIME)
        {
            if ((ret = bb_setUInt8(ba, 12)) == 0)
            {
                ret = var_getDateTime2(data->dateTime, ba);
            }
        }
        else if (data->vt == DLMS_DATA_TYPE_DATE)
        {
            if ((ret = bb_setUInt8(ba, 5)) == 0)
            {
                ret = var_getDate(data->dateTime, ba);
            }
        }
        else if (data->vt == DLMS_DATA_TYPE_TIME)
        {
            if ((ret = bb_setUInt8(ba, 4)) == 0)
            {
                ret = var_getTime(data->dateTime, ba);
            }
        }
        else
        {
            ret = var_setOctetString(ba, data);
        }
        break;
    case DLMS_DATA_TYPE_DATETIME:
    {
        ret = var_getDateTime2(data->dateTime, ba);
        break;
    }
    case DLMS_DATA_TYPE_DATE:
    {
        ret = var_getDate(data->dateTime, ba);
        break;
    }
    case DLMS_DATA_TYPE_TIME:
    {
        ret = var_getTime(data->dateTime, ba);
        break;
    }
    case DLMS_DATA_TYPE_BIT_STRING:
    {
        if ((ret = hlp_setObjectCount(data->bitArr->size, ba)) == 0)
        {
            ret = bb_set(ba, data->bitArr->data, ba_getByteCount(data->bitArr->size));
        }
        break;
    }
    default:
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
        assert(0);
#endif
        ret = DLMS_ERROR_CODE_INVALID_PARAMETER;
    }
    return ret;
}

//Get size in bytes.
int var_getSize(DLMS_DATA_TYPE vt)
{
    int nSize = -1;
    switch (vt)
    {
    case DLMS_DATA_TYPE_NONE:
        nSize = 0;
        break;
    case DLMS_DATA_TYPE_BOOLEAN:
    case DLMS_DATA_TYPE_INT8:
    case DLMS_DATA_TYPE_UINT8:
    case DLMS_DATA_TYPE_ENUM:
        nSize = 1;
        break;
    case DLMS_DATA_TYPE_INT16:
    case DLMS_DATA_TYPE_UINT16:
        nSize = 2;
        break;
    case DLMS_DATA_TYPE_INT32:
    case DLMS_DATA_TYPE_UINT32:
    case DLMS_DATA_TYPE_FLOAT32:
        nSize = 4;
        break;
    case DLMS_DATA_TYPE_INT64:
    case DLMS_DATA_TYPE_UINT64:
    case DLMS_DATA_TYPE_FLOAT64:
        nSize = 8;
        break;
    case DLMS_DATA_TYPE_BIT_STRING:
        break;
    case DLMS_DATA_TYPE_OCTET_STRING:
        break;
    case DLMS_DATA_TYPE_STRING:
    case DLMS_DATA_TYPE_STRING_UTF8:
        nSize = -1;
        break;
    case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
        break;
    case DLMS_DATA_TYPE_DATETIME:
        nSize = 12;
        break;
    case DLMS_DATA_TYPE_DATE:
        break;
    case DLMS_DATA_TYPE_TIME:
        break;
    case DLMS_DATA_TYPE_ARRAY:
        break;
    case DLMS_DATA_TYPE_STRUCTURE:
        break;
    case DLMS_DATA_TYPE_COMPACT_ARRAY:
        break;
    }
    return nSize;
}

//Convert variant value to integer.
int var_toInteger(dlmsVARIANT *data)
{
    if (data->vt == DLMS_DATA_TYPE_NONE)
    {
        return 0;
    }

    if (data->vt == DLMS_DATA_TYPE_BOOLEAN)
    {
        return data->boolVal ? 1 : 0;
    }
    if (data->vt == DLMS_DATA_TYPE_INT32)
    {
        return data->lVal;
    }
    if (data->vt == DLMS_DATA_TYPE_UINT32)
    {
        return data->ulVal;
    }
    if (data->vt == DLMS_DATA_TYPE_BINARY_CODED_DESIMAL)
    {
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
        assert(0);
#endif
    }
    if (data->vt == DLMS_DATA_TYPE_STRING_UTF8)
    {
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
        assert(0);
#endif
    }
    if (data->vt == DLMS_DATA_TYPE_INT8)
    {
        return data->cVal;
    }

    if (data->vt == DLMS_DATA_TYPE_INT16)
    {
        return data->iVal;
    }
    if (data->vt == DLMS_DATA_TYPE_UINT8)
    {
        return data->bVal;
    }
    if (data->vt == DLMS_DATA_TYPE_UINT16)
    {
        return data->uiVal;
    }
    if (data->vt == DLMS_DATA_TYPE_INT64)
    {
        //TODO:
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
        assert(0);
#endif
    }
    if (data->vt == DLMS_DATA_TYPE_UINT64)
    {
        //TODO:
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
        assert(0);
#endif
    }
    if (data->vt == DLMS_DATA_TYPE_ENUM)
    {
        return data->bVal;
    }
    if (data->vt == DLMS_DATA_TYPE_FLOAT32)
    {
        return (int)data->fltVal;
    }
    if (data->vt == DLMS_DATA_TYPE_FLOAT64)
    {
        return (int)data->dblVal;
    }
    if (data->vt == DLMS_DATA_TYPE_STRING)
    {
        return hlp_stringToInt((const char*)data->strVal);
    }
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
    assert(0);
#endif
    return 0;
}

char va_isAttached(variantArray* arr)
{
    return (arr->capacity & 0x8000) == 0x8000;
}

unsigned short va_getCapacity(variantArray* arr)
{
    return arr->capacity & 0x7FFF;
}

//Initialize variantArray.
void va_init(variantArray* arr)
{
    arr->capacity = 0;
    arr->data = NULL;
    arr->position = 0;
    arr->size = 0;
}

//Allocate new size for the array in bytes.
void va_capacity(variantArray* arr, unsigned short capacity)
{
    if (!va_isAttached(arr))
    {
        if (capacity == 0)
        {
            if (arr->capacity != 0)
            {
                gxfree(arr->data);
                arr->size = arr->position = 0;
            }
        }
        else
        {
            if (arr->capacity == 0)
            {
                arr->data = (void**)gxmalloc(capacity * sizeof(dlmsVARIANT*));
            }
            else
            {
                arr->data = (void**)gxrealloc(arr->data, capacity * sizeof(dlmsVARIANT *));
            }
        }
        arr->capacity = capacity;
    }
}

//Push new data to the variantArray.
int va_push(variantArray * arr, dlmsVARIANT *item)
{
    dlmsVARIANT ** p;
    if (!va_isAttached(arr))
    {
        if (arr->size >= arr->capacity)
        {
            arr->capacity += VECTOR_ARRAY_CAPACITY;
            if (arr->size == 0)
            {
                arr->data = (void**)gxmalloc(arr->capacity * sizeof(dlmsVARIANT *));
            }
            else
            {
                arr->data = (void**)gxrealloc(arr->data, arr->capacity * sizeof(dlmsVARIANT *));
            }
        }
    }
    if (va_getCapacity(arr) <= arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    p = (dlmsVARIANT **)arr->data;
    p[arr->size] = item;
    ++arr->size;
    return 0;
}

void va_clear(
    variantArray* arr)
{
    int pos;
    unsigned char attached = va_isAttached(arr);
    if (arr->data != NULL && !attached)
    {
        for (pos = 0; pos != arr->size; ++pos)
        {
            var_clear((dlmsVARIANT *)arr->data[pos]);
            gxfree(arr->data[pos]);
        }
        gxfree(arr->data);
        arr->data = NULL;
    }
    if (!attached)
    {
        arr->capacity = 0;
    }
    arr->size = 0;
    arr->position = 0;
}

void va_attach(
    variantArray* trg,
    variantArray* src)
{
    trg->capacity = src->capacity;
    trg->data = src->data;
    trg->position = src->position;
    trg->size = src->size;
    src->data = NULL;
    src->position = src->size = src->capacity = 0;
}

//Get item from variant array.
int va_get(variantArray* arr, dlmsVARIANT ** item)
{
    dlmsVARIANT ** p;
    if (arr->position >= arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    p = (dlmsVARIANT **)arr->data;
    *item = p[arr->position];
    ++arr->position;
    return DLMS_ERROR_CODE_OK;
}


//Get item from variant array by index.
int va_getByIndex(variantArray* arr, int index, dlmsVARIANT ** item)
{
    dlmsVARIANT ** p;
    if (index >= arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    p = (dlmsVARIANT **)arr->data;
    *item = p[index];
    return DLMS_ERROR_CODE_OK;
}

int va_copyArray(
    variantArray* target,
    variantArray* source)
{
    int ret = DLMS_ERROR_CODE_OK;
    dlmsVARIANT *tmp, *tmp2;
    int pos;
    va_clear(target);
    for (pos = 0; pos != source->size; ++pos)
    {
        ret = va_get(source, &tmp);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            return ret;
        }
        tmp2 = (dlmsVARIANT *)gxmalloc(sizeof(dlmsVARIANT));
        var_init(tmp2);
        ret = var_copy(tmp2, tmp);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            return ret;
        }
        va_push(target, tmp2);
    }
    return ret;
}

//Note! var_toString do not clear existing bytearray.
int var_toString(dlmsVARIANT *item, gxByteBuffer* value)
{
    int ret = DLMS_ERROR_CODE_OK;
    dlmsVARIANT tmp;
    var_init(&tmp);
    ret = var_copy(&tmp, item);
    if (ret == 0)
    {
        ret = var_changeType(&tmp, DLMS_DATA_TYPE_STRING);
        if (ret == 0 && tmp.strVal != NULL)
        {
            bb_set(value, tmp.strVal->data, tmp.strVal->size);
        }
    }
    var_clear(&tmp);
    return ret;
}


//Note! va_toString do not clear existing bytearray.
int va_toString(
    variantArray *items,
    gxByteBuffer* ba)
{
    dlmsVARIANT * it;
    int pos, ret = DLMS_ERROR_CODE_OK;
    for (pos = 0; pos != items->size; ++pos)
    {
        if ((ret = va_getByIndex(items, pos, &it)) != 0)
        {
            return ret;
        }
        if (pos != 0)
        {
            bb_addString(ba, ", ");
        }
        if ((ret = var_toString(it, ba)) != 0)
        {
            return ret;
        }
    }
    return ret;
}

static int convert(dlmsVARIANT *item, DLMS_DATA_TYPE type)
{
    unsigned char ch;
    int ret, fromSize, toSize;
    unsigned short pos;
    char buff[250];
    dlmsVARIANT tmp, tmp3;
    dlmsVARIANT *it;
    if (item->vt == type)
    {
        return DLMS_ERROR_CODE_OK;
    }
    var_init(&tmp);
    var_init(&tmp3);
    ret = var_copy(&tmp, item);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    var_clear(item);
    if (type == DLMS_DATA_TYPE_STRING)
    {
        item->strVal = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
        bb_init(item->strVal);
        if (tmp.vt == DLMS_DATA_TYPE_ARRAY || tmp.vt == DLMS_DATA_TYPE_STRUCTURE)
        {
            bb_setUInt8(item->strVal, '{');
            for (pos = 0; pos != tmp.Arr->size; ++pos)
            {
                ret = va_get(tmp.Arr, &it);
                if (ret != DLMS_ERROR_CODE_OK)
                {
                    return ret;
                }
                if (pos != 0)
                {
                    bb_setUInt8(item->strVal, ',');
                    bb_setUInt8(item->strVal, ' ');
                }
                ret = var_copy(&tmp3, it);
                if (ret != DLMS_ERROR_CODE_OK)
                {
                    return ret;
                }
                ret = var_toString(&tmp3, item->strVal);
                var_clear(&tmp3);
                if (ret != DLMS_ERROR_CODE_OK)
                {
                    var_clear(&tmp);
                    return ret;
                }
            }
            bb_setUInt8(item->strVal, '}');
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_BOOLEAN)
        {
            if (tmp.boolVal == 0)
            {
                bb_addString(item->strVal, "False");
            }
            else
            {
                bb_addString(item->strVal, "True");
            }
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_INT32)
        {
            hlp_intToString(buff, 250, tmp.lVal, 1);
            bb_addString(item->strVal, buff);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_UINT32)
        {
            hlp_intToString(buff, 250, tmp.ulVal, 0);
            bb_addString(item->strVal, buff);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_INT8)
        {
            hlp_intToString(buff, 250, tmp.cVal, 1);
            bb_addString(item->strVal, buff);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_INT16)
        {
            hlp_intToString(buff, 250, tmp.iVal, 1);
            bb_addString(item->strVal, buff);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_UINT8)
        {
            hlp_intToString(buff, 250, tmp.bVal, 0);
            bb_addString(item->strVal, buff);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_UINT16)
        {
            hlp_intToString(buff, 250, tmp.uiVal, 0);
            bb_addString(item->strVal, buff);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_INT64)
        {
            hlp_int64ToString(buff, 250, tmp.uiVal, 1);
            bb_addString(item->strVal, buff);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_UINT64)
        {
            hlp_int64ToString(buff, 250, tmp.ullVal, 0);
            bb_addString(item->strVal, buff);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_ENUM)
        {
            hlp_intToString(buff, 250, tmp.bVal, 0);
            bb_addString(item->strVal, buff);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_FLOAT32)
        {
#ifndef GX_DLMS_MICROCONTROLLER
#if _MSC_VER > 1000
            sprintf_s(buff, 250, "%f", tmp.fltVal);
#else
            sprintf(buff, "%f", tmp.fltVal);
#endif
            bb_addString(item->strVal, buff);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
#else
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
#endif //GX_DLMS_MICROCONTROLLER
        }
        else if (tmp.vt == DLMS_DATA_TYPE_FLOAT64)
        {
#ifndef GX_DLMS_MICROCONTROLLER
#if _MSC_VER > 1000
            sprintf_s(buff, 250, "%lf", tmp.dblVal);
#else
            sprintf(buff, "%lf", tmp.dblVal);
#endif
            bb_addString(item->strVal, buff);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
#else
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
#endif //GX_DLMS_MICROCONTROLLER
        }
        else if (tmp.vt == DLMS_DATA_TYPE_BIT_STRING)
        {
            char* str = ba_toString(tmp.bitArr);
            bb_attachString(item->strVal, str);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_DATETIME)
        {
#ifndef GX_DLMS_MICROCONTROLLER
            time_toString(tmp.dateTime, item->strVal);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
#else
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
#endif //GX_DLMS_MICROCONTROLLER

        }
        else if (tmp.vt == DLMS_DATA_TYPE_OCTET_STRING)
        {
            if (tmp.byteArr != NULL)
            {
                for (pos = 0; pos != tmp.byteArr->size; ++pos)
                {
                    ret = bb_getUInt8(tmp.byteArr, &ch);
                    if (ret != DLMS_ERROR_CODE_OK)
                    {
                        return ret;
                    }
                    if (pos != 0)
                    {
                        bb_setUInt8(item->strVal, '.');
                    }
                    hlp_intToString(buff, 4, ch, 0);
                    bb_addString(item->strVal, buff);
                }
            }
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (tmp.vt == DLMS_DATA_TYPE_NONE)
        {
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else
        {
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
        }
    }
    else if (item->vt == DLMS_DATA_TYPE_STRING)
    {
        if (type == DLMS_DATA_TYPE_BOOLEAN)
        {
            item->boolVal = strcmp((char*)tmp.strVal->data, "False") == 0 ? 0 : 1;
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (type == DLMS_DATA_TYPE_INT32)
        {
            item->lVal = hlp_stringToInt((char*)tmp.strVal->data);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (type == DLMS_DATA_TYPE_UINT32)
        {
            item->ulVal = hlp_stringToInt((char*)tmp.strVal->data) & 0xFFFFFFFF;
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (type == DLMS_DATA_TYPE_INT8)
        {
            item->cVal = (char)hlp_stringToInt((char*)tmp.strVal->data);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (type == DLMS_DATA_TYPE_INT16)
        {
            item->iVal = (short)hlp_stringToInt((char*)tmp.strVal->data);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (type == DLMS_DATA_TYPE_UINT8)
        {
            item->bVal = (unsigned char)hlp_stringToInt((char*)tmp.strVal->data);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (type == DLMS_DATA_TYPE_UINT16)
        {
            item->uiVal = (unsigned short)hlp_stringToInt((char*)tmp.strVal->data);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (type == DLMS_DATA_TYPE_INT64)
        {
            item->llVal = hlp_stringToInt64((char*)tmp.strVal->data);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (type == DLMS_DATA_TYPE_UINT64)
        {
            item->ullVal = (unsigned long long) hlp_stringToInt64((char*)tmp.strVal->data);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (type == DLMS_DATA_TYPE_ENUM)
        {
            item->bVal = (unsigned char)hlp_stringToInt((char*)tmp.strVal->data);
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
#ifndef GX_DLMS_MICROCONTROLLER
        else if (type == DLMS_DATA_TYPE_FLOAT32)
        {
#if _MSC_VER > 1000
            sscanf_s((char*)tmp.strVal->data, "%f", &item->fltVal);
#else
            sscanf((char*)tmp.strVal->data, "%f", &item->fltVal);
#endif
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
        else if (type == DLMS_DATA_TYPE_FLOAT64)
        {
#if _MSC_VER > 1000
            sscanf_s((char*)tmp.strVal->data, "%lf", &item->dblVal);
#else
            sscanf((char*)tmp.strVal->data, "%lf", &item->dblVal);
#endif
            item->vt = type;
            var_clear(&tmp);
            return DLMS_ERROR_CODE_OK;
        }
#endif //GX_DLMS_MICROCONTROLLER
        else if (type == DLMS_DATA_TYPE_OCTET_STRING)
        {
            char* pBuff = (char*)tmp.strVal->data;
            item->byteArr = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
            bb_init(item->byteArr);
            bb_addHexString(item->byteArr, pBuff);
            item->vt = type;
            var_clear(&tmp);
            bb_trim(item->byteArr);
            return DLMS_ERROR_CODE_OK;
        }
        return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
    }
    fromSize = var_getSize(tmp.vt);
    toSize = var_getSize(item->vt);
    //If we try to change bigger valut to smaller check that value is not too big.
    //Example Int16 to Int8.
    if (fromSize > toSize)
    {
        unsigned char* pValue = &tmp.bVal;
        for (pos = (unsigned char)toSize; pos != (unsigned char)fromSize; ++pos)
        {
            if (pValue[pos] != 0)
            {
                return DLMS_ERROR_CODE_INVALID_PARAMETER;
            }
        }
    }
    if (fromSize > toSize)
    {
        memcpy(&item->bVal, &tmp.bVal, toSize);
    }
    else
    {
        memset(&item->bVal, 0, toSize);
        memcpy(&item->bVal, &tmp.bVal, fromSize);
    }
    item->vt = type;
    var_clear(&tmp);
    return DLMS_ERROR_CODE_OK;
}

int var_changeType(dlmsVARIANT *value, DLMS_DATA_TYPE newType)
{
    if (newType == value->vt)
    {
        return DLMS_ERROR_CODE_OK;
    }
    if (newType == DLMS_DATA_TYPE_NONE)
    {
        return var_clear(value);
    }
    if (value->vt == DLMS_DATA_TYPE_ARRAY && newType == DLMS_DATA_TYPE_OCTET_STRING)
    {
        return DLMS_ERROR_CODE_OK;
    }
    if (value->vt == DLMS_DATA_TYPE_STRING)
    {
        return convert(value, newType);
    }
    switch (newType)
    {
    case DLMS_DATA_TYPE_STRING:
    case DLMS_DATA_TYPE_BOOLEAN:
    case DLMS_DATA_TYPE_INT32:
    case DLMS_DATA_TYPE_UINT32:
    case DLMS_DATA_TYPE_INT8:
    case DLMS_DATA_TYPE_INT16:
    case DLMS_DATA_TYPE_UINT8:
    case DLMS_DATA_TYPE_UINT16:
    case DLMS_DATA_TYPE_INT64:
    case DLMS_DATA_TYPE_UINT64:
    case DLMS_DATA_TYPE_ENUM:
#ifndef GX_DLMS_MICROCONTROLLER
    case DLMS_DATA_TYPE_FLOAT32:
    case DLMS_DATA_TYPE_FLOAT64:
#endif //GX_DLMS_MICROCONTROLLER
        return convert(value, newType);
        break;
    default:
        //Handled later.
        break;
    }
    switch (value->vt)
    {
    case DLMS_DATA_TYPE_BOOLEAN:
        return DLMS_ERROR_CODE_INVALID_PARAMETER;
    case DLMS_DATA_TYPE_BIT_STRING:
    {
        return DLMS_ERROR_CODE_INVALID_PARAMETER;
    }
    case DLMS_DATA_TYPE_INT32:
        switch (newType)
        {
        case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_UINT32:
        switch (newType)
        {
        case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_OCTET_STRING:
        switch (newType)
        {
        case DLMS_DATA_TYPE_DATETIME:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        case DLMS_DATA_TYPE_DATE:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        case DLMS_DATA_TYPE_TIME:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
        switch (newType)
        {
        case DLMS_DATA_TYPE_INT32:
            break;
        case DLMS_DATA_TYPE_UINT32:
            break;
        case DLMS_DATA_TYPE_STRING:
            break;
        case DLMS_DATA_TYPE_INT8:
            break;
        case DLMS_DATA_TYPE_INT16:
            break;
        case DLMS_DATA_TYPE_UINT8:
            break;
        case DLMS_DATA_TYPE_UINT16:
            break;
        case DLMS_DATA_TYPE_INT64:
            break;
        case DLMS_DATA_TYPE_UINT64:
            break;
        case DLMS_DATA_TYPE_ENUM:
            break;
        case DLMS_DATA_TYPE_FLOAT32:
            break;
        case DLMS_DATA_TYPE_FLOAT64:
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_INT8:
        switch (newType)
        {
        case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_INT16:
        switch (newType)
        {
        case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_UINT8:
        switch (newType)
        {
        case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_UINT16:
        switch (newType)
        {
        case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_INT64:
        switch (newType)
        {
        case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_UINT64:
        switch (newType)
        {
        case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_ENUM:
        switch (newType)
        {
        case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_FLOAT32:
        switch (newType)
        {
        case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_FLOAT64:
        switch (newType)
        {
        case DLMS_DATA_TYPE_BINARY_CODED_DESIMAL:
            return DLMS_ERROR_CODE_NOT_IMPLEMENTED;
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_DATETIME:
        switch (newType)
        {
        case DLMS_DATA_TYPE_OCTET_STRING:
            break;
        case DLMS_DATA_TYPE_STRING:
            break;
        case DLMS_DATA_TYPE_DATE:
            break;
        case DLMS_DATA_TYPE_TIME:
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_DATE:
        switch (newType)
        {
        case DLMS_DATA_TYPE_OCTET_STRING:
            break;
        case DLMS_DATA_TYPE_STRING:
            break;
        case DLMS_DATA_TYPE_DATETIME:
            break;
        case DLMS_DATA_TYPE_DATE:
            break;
        case DLMS_DATA_TYPE_TIME:
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_TIME:
        switch (newType)
        {
        case DLMS_DATA_TYPE_OCTET_STRING:
            break;
        case DLMS_DATA_TYPE_STRING:
            break;
        case DLMS_DATA_TYPE_DATETIME:
            break;
        case DLMS_DATA_TYPE_DATE:
            break;
        case DLMS_DATA_TYPE_TIME:
            break;
        default:
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
        break;
    case DLMS_DATA_TYPE_ARRAY:
    case DLMS_DATA_TYPE_STRUCTURE:
    case DLMS_DATA_TYPE_COMPACT_ARRAY:
    default:
        return DLMS_ERROR_CODE_INVALID_PARAMETER;
    }
    return DLMS_ERROR_CODE_OK;
}

//copy variant.
int var_copy(dlmsVARIANT *target, dlmsVARIANT *source)
{
    dlmsVARIANT *it;
    dlmsVARIANT *item;
    int ret = DLMS_ERROR_CODE_OK, pos;
    unsigned char attaced = 0;
    if ((target->vt == DLMS_DATA_TYPE_ARRAY || target->vt == DLMS_DATA_TYPE_STRUCTURE) && va_isAttached(target->Arr))
    {
        attaced = 1;
        target->Arr->position = 0;
    }
    else
    {
        ret = var_clear(target);
        if (ret != DLMS_ERROR_CODE_OK)
        {
            return ret;
        }
        target->vt = source->vt;
    }
    if (source->vt == DLMS_DATA_TYPE_STRING)
    {
        if (source->strVal != NULL)
        {
            target->strVal = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
            bb_init(target->strVal);
            bb_set(target->strVal, source->strVal->data, source->strVal->size);
        }
    }
    else if (source->vt == DLMS_DATA_TYPE_OCTET_STRING)
    {
        if (source->byteArr != 0)
        {
            target->byteArr = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
            bb_init(target->byteArr);
            bb_set(target->byteArr, source->byteArr->data, source->byteArr->size);
        }
    }
    else if (source->vt == DLMS_DATA_TYPE_ARRAY ||
        source->vt == DLMS_DATA_TYPE_STRUCTURE)
    {
        if (source->Arr != NULL && source->Arr->size != 0)
        {
            if (target->Arr == NULL)
            {
                target->Arr = (variantArray*)gxmalloc(sizeof(variantArray));
                va_init(target->Arr);
            }
            va_capacity(target->Arr, source->Arr->size);
            for (pos = 0; pos != source->Arr->size; ++pos)
            {
                if ((ret = va_getByIndex(source->Arr, pos, &it)) != DLMS_ERROR_CODE_OK)
                {
                    return ret;
                }
                if (attaced)
                {
                    if ((ret = va_getByIndex(target->Arr, pos, &item)) != DLMS_ERROR_CODE_OK ||
                        (ret = var_copy(item, it)) != DLMS_ERROR_CODE_OK)
                    {
                        return ret;
                    }
                }
                else
                {
                    item = (dlmsVARIANT *)gxmalloc(sizeof(dlmsVARIANT));
                    ret = var_init(item);
                    if (ret != DLMS_ERROR_CODE_OK)
                    {
                        return ret;
                    }
                    ret = var_copy(item, it);
                    if (ret != DLMS_ERROR_CODE_OK)
                    {
                        return ret;
                    }
                    va_push(target->Arr, item);
                }
            }
        }
    }
    else if (source->vt == DLMS_DATA_TYPE_DATETIME)
    {
        ret = var_setDateTime(target, source->dateTime);
    }
    else if (source->vt == DLMS_DATA_TYPE_DATE)
    {
        ret = var_setDate(target, source->dateTime);
    }
    else if (source->vt == DLMS_DATA_TYPE_TIME)
    {
        ret = var_setTime(target, source->dateTime);
    }
    else if (source->vt == DLMS_DATA_TYPE_BIT_STRING)
    {
        target->bitArr = (bitArray*)gxmalloc(sizeof(bitArray));
        ba_init(target->bitArr);
        ret = ba_copy(target->bitArr, source->bitArr->data, (unsigned short)source->bitArr->size);
    }
    else
    {
        ret = var_getSize(source->vt);
        if (ret > 0)
        {
            memcpy(&target->bVal, &source->bVal, ret);
        }
        ret = 0;
    }
    return ret;
}

int var_setDateTime(dlmsVARIANT *target, gxtime* value)
{
    int ret;
    ret = var_clear(target);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    target->dateTime = (gxtime*)gxmalloc(sizeof(gxtime));
    time_copy(target->dateTime, value);
    target->vt = DLMS_DATA_TYPE_DATETIME;
    return ret;
}

int var_setDate(dlmsVARIANT *target, gxtime* value)
{
    int ret;
    ret = var_clear(target);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    target->dateTime = (gxtime*)gxmalloc(sizeof(gxtime));
    time_copy(target->dateTime, value);
    target->vt = DLMS_DATA_TYPE_DATE;
    return ret;
}

int var_setTime(dlmsVARIANT *target, gxtime* value)
{
    int ret;
    ret = var_clear(target);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    target->dateTime = (gxtime*)gxmalloc(sizeof(gxtime));
    time_copy(target->dateTime, value);
    target->vt = DLMS_DATA_TYPE_TIME;
    return ret;
}

int var_setDateTimeAsOctetString(
    dlmsVARIANT *target,
    gxtime* value)
{
    int ret;
    ret = var_clear(target);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    target->byteArr = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
    bb_init(target->byteArr);
    bb_capacity(target->byteArr, 12);
    if ((ret = var_getDateTime2(value, target->byteArr)) != 0)
    {
        return ret;
    }
    target->vt = DLMS_DATA_TYPE_OCTET_STRING;
    return ret;
}

int var_setDateAsOctetString(
    dlmsVARIANT *target,
    gxtime* value)
{
    int ret;
    ret = var_clear(target);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    target->byteArr = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
    bb_init(target->byteArr);
    bb_capacity(target->byteArr, 5);
    if ((ret = var_getDate(value, target->byteArr)) != 0)
    {
        return ret;
    }
    target->vt = DLMS_DATA_TYPE_OCTET_STRING;
    return ret;
}

int var_setTimeAsOctetString(
    dlmsVARIANT *target,
    gxtime* value)
{
    int ret;
    ret = var_clear(target);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    target->byteArr = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
    bb_init(target->byteArr);
    bb_capacity(target->byteArr, 4);
    if ((ret = var_getTime(value, target->byteArr)) != 0)
    {
        return ret;
    }
    target->vt = DLMS_DATA_TYPE_OCTET_STRING;
    return 0;
}

int var_setBoolean(dlmsVARIANT *target, char value)
{
    int ret;
    ret = var_clear(target);
    if (ret != DLMS_ERROR_CODE_OK)
    {
        return ret;
    }
    target->boolVal = value;
    target->vt = DLMS_DATA_TYPE_BOOLEAN;
    return ret;
}

void var_attach(
    dlmsVARIANT *target,
    gxByteBuffer * source)
{
    target->byteArr = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
    target->byteArr->data = source->data;
    target->byteArr->capacity = source->capacity;
    target->byteArr->size = source->size;
    target->byteArr->position = source->position;
    source->data = 0;
    source->size = source->position = source->capacity = 0;
    target->vt = DLMS_DATA_TYPE_OCTET_STRING;
}

int var_getDateTime(dlmsVARIANT *target, gxtime* value)
{
    if (target->vt == DLMS_DATA_TYPE_NONE)
    {
        time_clear(value);
    }
    else if (target->vt == DLMS_DATA_TYPE_DATETIME ||
        target->vt == DLMS_DATA_TYPE_DATE ||
        target->vt == DLMS_DATA_TYPE_TIME)
    {
        value->daylightSavingsBegin = target->dateTime->daylightSavingsBegin;
        value->daylightSavingsEnd = target->dateTime->daylightSavingsEnd;
        value->skip = target->dateTime->skip;
        value->status = target->dateTime->status;
        value->value = target->dateTime->value;
    }
    else
    {
        return DLMS_ERROR_CODE_INVALID_PARAMETER;
    }
    return DLMS_ERROR_CODE_OK;
}

double var_toDouble(dlmsVARIANT *target)
{
    switch (target->vt)
    {
    case DLMS_DATA_TYPE_NONE:
    {
        return 0;
    }
    case DLMS_DATA_TYPE_BOOLEAN:
    {
        return target->boolVal ? 1 : 0;
    }
    case DLMS_DATA_TYPE_INT32:
    {
        return target->lVal;
    }
    case DLMS_DATA_TYPE_UINT32:
    {
        return target->ulVal;
    }
    case DLMS_DATA_TYPE_STRING_UTF8:
    {
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
        assert(0);
#endif
        return 0;
    }
    case DLMS_DATA_TYPE_STRING:
    {
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
        assert(0);
#endif
        return 0;
    }
    case DLMS_DATA_TYPE_INT8:
    {
        return target->cVal;
    }
    case DLMS_DATA_TYPE_INT16:
    {
        return target->iVal;
    }
    case DLMS_DATA_TYPE_UINT8:
    {
        return target->bVal;
    }
    case DLMS_DATA_TYPE_UINT16:
    {
        return target->uiVal;
    }
    case DLMS_DATA_TYPE_INT64:
    {
        return (double)target->llVal;
    }
    case DLMS_DATA_TYPE_UINT64:
    {
        return (double)target->ullVal;
    }
    case DLMS_DATA_TYPE_ENUM:
    {
        return target->bVal;
    }
    case DLMS_DATA_TYPE_FLOAT32:
    {
        return target->fltVal;
    }
    case DLMS_DATA_TYPE_FLOAT64:
    {
        return target->dblVal;
    }
    default:
        break;
    }
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
    assert(0);
#endif
    return 0;
}

#ifndef GX_DLMS_MICROCONTROLLER
//Print content of the variant to cout.
int var_print(const char* format, dlmsVARIANT *target)
{
    int ret = DLMS_ERROR_CODE_OK;
    dlmsVARIANT tmp;
    var_init(&tmp);
    ret = var_copy(&tmp, target);
    if (ret == 0)
    {
        ret = var_changeType(&tmp, DLMS_DATA_TYPE_STRING);
        if (ret == 0 && tmp.strVal != NULL)
        {
            if (format == NULL)
            {
                printf(format, tmp.strVal->data);
            }
        }
    }
    var_clear(&tmp);
    return ret;
}

int va_print(
    variantArray *items)
{
    dlmsVARIANT * it;
    int pos, ret = DLMS_ERROR_CODE_OK;
    for (pos = 0; pos != items->size; ++pos)
    {
        if ((ret = va_getByIndex(items, pos, &it)) != 0)
        {
            return ret;
        }

        if ((ret = var_print(NULL, it)) != 0)
        {
            return ret;
        }
    }
    return ret;
}

#endif //GX_DLMS_MICROCONTROLLER
