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

#include "../include/gxmem.h"
#ifndef GX_DLMS_MICROCONTROLLER
#include <stdio.h> //printf needs this or error is generated.
#endif //GX_DLMS_MICROCONTROLLER

#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
#include <assert.h>
#if _MSC_VER > 1400
#include <crtdbg.h>
#endif
#endif
#include <string.h>
#include "../include/errorcodes.h"
#include "../include/bytebuffer.h"
#include "../include/helpers.h"

char bb_isAttached(gxByteBuffer* arr)
{
#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
    //If byte buffer is attached.
    return (arr->capacity & 0x80000000) == 0x80000000;
#else
    return (arr->capacity & 0x8000) == 0x8000;
#endif
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
unsigned long bb_getCapacity(gxByteBuffer* arr)
#else
unsigned short bb_getCapacity(gxByteBuffer* arr)
#endif
{
#if !defined(GX_DLMS_MICROCONTROLLER)&& (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
    //If byte buffer is attached.
    return arr->capacity & 0x7FFFFFFF;
#else
    return arr->capacity & 0x7FFF;
#endif
}


#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
unsigned long bb_size(gxByteBuffer* arr)
#else
unsigned short bb_size(gxByteBuffer* arr)
#endif
{
    if (arr == NULL)
    {
        return 0;
    }
    return arr->size;
}

int bb_init(
    gxByteBuffer* arr)
{
    arr->capacity = 0;
    arr->data = NULL;
    arr->position = 0;
    arr->size = 0;
    return 0;
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
int bb_capacity(
    gxByteBuffer* arr,
    unsigned long capacity)
#else
int bb_capacity(
    gxByteBuffer* arr,
    unsigned short capacity)
#endif
{
    //Capacity can't change if it's attached.
    if (!bb_isAttached(arr))
    {
        if (capacity == 0)
        {
            if (arr->data != NULL)
            {
                gxfree(arr->data);
                arr->data = NULL;
                arr->size = 0;
            }
        }
        else
        {
            if (arr->capacity == 0)
            {
                arr->data = (unsigned char*)gxmalloc(capacity);
            }
            else
            {
                arr->data = (unsigned char*)gxrealloc(arr->data, capacity);
                //If not enought memory available.
                if (arr->data == NULL)
                {
                    return DLMS_ERROR_CODE_OUTOFMEMORY;
                }
            }
            if (arr->size > capacity)
            {
                arr->size = capacity;
            }
        }
        arr->capacity = capacity;
    }
    return DLMS_ERROR_CODE_OK;
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
void bb_zero(
    gxByteBuffer* arr,
    unsigned long index,
    unsigned long count)
#else
void bb_zero(
    gxByteBuffer* arr,
    unsigned short index,
    unsigned short count)
#endif
{
    if (index + count > arr->capacity)
    {
        bb_capacity(arr, index + count);
    }
    if (arr->size < index + count)
    {
        arr->size = index + count;
    }
    memset(arr->data + index, 0, count);
}

int bb_setUInt8(
    gxByteBuffer * arr,
    unsigned char item)
{
    int ret = bb_setUInt8ByIndex(arr, arr->size, item);
    if (ret == 0)
    {
        ++arr->size;
    }
    return ret;
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
int bb_insertUInt8(
    gxByteBuffer * arr,
    unsigned long index,
    unsigned char item)
#else
int bb_insertUInt8(
    gxByteBuffer * arr,
    unsigned short index,
    unsigned char item)
#endif
{
    int ret;
    if ((ret = bb_move(arr, index, index + 1, arr->size)) == 0)
    {
        ret = bb_setUInt8ByIndex(arr, index, item);
    }
    return ret;
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
int bb_setUInt8ByIndex(
    gxByteBuffer *arr,
    unsigned long index,
    unsigned char item)
#else
int bb_setUInt8ByIndex(
    gxByteBuffer *arr,
    unsigned short index,
    unsigned char item)
#endif
{
    if (!bb_isAttached(arr) && (arr->capacity == 0 || index >= arr->capacity))
    {
        arr->capacity += VECTOR_CAPACITY;
        if (arr->size == 0)
        {
            arr->data = (unsigned char*)gxmalloc(arr->capacity);
        }
        else
        {
            arr->data = (unsigned char*)gxrealloc(arr->data, arr->capacity);
            if (arr->data == NULL)
            {
                return DLMS_ERROR_CODE_OUTOFMEMORY;
            }
        }
    }
    if (bb_getCapacity(arr) <= index)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    arr->data[index] = item;
    return 0;
}


int bb_setUInt16(
    gxByteBuffer *arr,
    unsigned short item)
{
    if (!bb_isAttached(arr) && (arr->capacity == 0 || arr->size + 2 > arr->capacity))
    {
        arr->capacity += VECTOR_CAPACITY;
        if (arr->size == 0)
        {
            arr->data = (unsigned char*)gxmalloc(arr->capacity);
        }
        else
        {
            arr->data = (unsigned char*)gxrealloc(arr->data, arr->capacity);
        }
    }
    if (bb_getCapacity(arr) < arr->size + 2)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    arr->data[arr->size] = (item >> 8) & 0xFF;
    arr->data[arr->size + 1] = item & 0xFF;
    arr->size += 2;
    return 0;
}

int bb_setUInt32(
    gxByteBuffer* arr,
    unsigned long item)
{
    int ret = bb_setUInt32ByIndex(arr, arr->size, item);
    if (ret == 0)
    {
        arr->size += 4;
    }
    return ret;
}

int bb_setUInt32ByIndex(
    gxByteBuffer* arr,
    unsigned long index,
    unsigned long item)
{
    if (!bb_isAttached(arr) && (arr->capacity == 0 || index + 4 > arr->capacity))
    {
        arr->capacity += VECTOR_CAPACITY;
        if (arr->data == NULL)
        {
            arr->data = (unsigned char*)gxmalloc(arr->capacity);
        }
        else
        {
            arr->data = (unsigned char*)gxrealloc(arr->data, arr->capacity);
        }
    }
    if (bb_getCapacity(arr) < index + 4)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    PUT32(arr->data + index, item);
    return 0;
}

int bb_setUInt64(
    gxByteBuffer* arr,
    unsigned long long item)
{
    if (!bb_isAttached(arr) && (arr->capacity == 0 || arr->size + 8 > arr->capacity))
    {
        arr->capacity += VECTOR_CAPACITY;
        if (arr->data == NULL)
        {
            arr->data = (unsigned char*)gxmalloc(arr->capacity);
        }
        else
        {
            arr->data = (unsigned char*)gxrealloc(arr->data, arr->capacity);
        }
    }
    if (bb_getCapacity(arr) < arr->size + 8)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    ((unsigned char*)arr->data)[arr->size + 7] = item & 0xFF;
    item >>= 8;
    ((unsigned char*)arr->data)[arr->size + 6] = item & 0xFF;
    item >>= 8;
    ((unsigned char*)arr->data)[arr->size + 5] = item & 0xFF;
    item >>= 8;
    ((unsigned char*)arr->data)[arr->size + 4] = item & 0xFF;
    item >>= 8;
    ((unsigned char*)arr->data)[arr->size + 3] = item & 0xFF;
    item >>= 8;
    ((unsigned char*)arr->data)[arr->size + 2] = item & 0xFF;
    item >>= 8;
    ((unsigned char*)arr->data)[arr->size + 1] = item & 0xFF;
    item >>= 8;
    ((unsigned char*)arr->data)[arr->size] = item & 0xFF;
    /*
    ((unsigned char*)arr->data)[arr->size] = (unsigned char)((item >> 56) & 0xFF);
    ((unsigned char*)arr->data)[arr->size + 1] = (item >> 48) & 0xFF;
    ((unsigned char*)arr->data)[arr->size + 2] = (item >> 40) & 0xFF;
    ((unsigned char*)arr->data)[arr->size + 3] = (item >> 32) & 0xFF;
    ((unsigned char*)arr->data)[arr->size + 4] = (item >> 24) & 0xFF;
    ((unsigned char*)arr->data)[arr->size + 5] = (item >> 16) & 0xFF;
    ((unsigned char*)arr->data)[arr->size + 6] = (item >> 8) & 0xFF;
    ((unsigned char*)arr->data)[arr->size + 7] = item & 0xFF;
*/
    arr->size += 8;
    return 0;
}

#ifndef GX_DLMS_MICROCONTROLLER
int bb_setFloat(
    gxByteBuffer* arr,
    float value)
{
    typedef union
    {
        float value;
        char b[sizeof(float)];
    } HELPER;

    HELPER tmp;
    tmp.value = value;
    if (!bb_isAttached(arr) && (arr->capacity == 0 || arr->size + 4 > arr->capacity))
    {
        arr->capacity += VECTOR_CAPACITY;
        if (arr->data == NULL)
        {
            arr->data = (unsigned char*)gxmalloc(arr->capacity);
        }
        else
        {
            arr->data = (unsigned char*)gxrealloc(arr->data, arr->capacity);
        }
    }
    if (bb_getCapacity(arr) < arr->size + 4)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    arr->data[arr->size] = tmp.b[3];
    arr->data[arr->size + 1] = tmp.b[2];
    arr->data[arr->size + 2] = tmp.b[1];
    arr->data[arr->size + 3] = tmp.b[0];
    arr->size += 4;
    return 0;
}

int bb_setDouble(
    gxByteBuffer* arr,
    double value)
{
    typedef union
    {
        double value;
        char b[sizeof(double)];
    } HELPER;

    HELPER tmp;
    tmp.value = value;
    if (!bb_isAttached(arr) && (arr->capacity == 0 || arr->size + 8 > arr->capacity))
    {
        arr->capacity += VECTOR_CAPACITY;
        if (arr->data == NULL)
        {
            arr->data = (unsigned char*)gxmalloc(arr->capacity);
        }
        else
        {
            arr->data = (unsigned char*)gxrealloc(arr->data, arr->capacity);
        }
    }
    if (bb_getCapacity(arr) < arr->size + 8)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    arr->data[arr->size] = tmp.b[7];
    arr->data[arr->size + 1] = tmp.b[6];
    arr->data[arr->size + 2] = tmp.b[5];
    arr->data[arr->size + 3] = tmp.b[4];
    arr->data[arr->size + 4] = tmp.b[3];
    arr->data[arr->size + 5] = tmp.b[2];
    arr->data[arr->size + 6] = tmp.b[1];
    arr->data[arr->size + 7] = tmp.b[0];
    arr->size += 8;
    return 0;
}
#endif //GX_DLMS_MICROCONTROLLER


int bb_setInt8(
    gxByteBuffer* arr,
    char item)
{
    return bb_setUInt8(arr, (unsigned char)item);
}

int bb_setInt16(
    gxByteBuffer* arr,
    short item)
{
    return bb_setUInt16(arr, (unsigned short)item);
}

int bb_setInt32(
    gxByteBuffer* arr,
    long item)
{
    return bb_setUInt32(arr, (unsigned long)item);
}

int bb_setInt64(
    gxByteBuffer* arr,
    long long item)
{
    return bb_setUInt64(arr, (unsigned long long) item);
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
int bb_set(
    gxByteBuffer* arr,
    const unsigned char* pSource,
    unsigned long count)
#else
int bb_set(
    gxByteBuffer* arr,
    const unsigned char* pSource,
    unsigned short count)
#endif
{
    if (!bb_isAttached(arr) && (arr->size + count > arr->capacity))
    {
        //First time data is reserved only for the added data.
        if (arr->capacity == 0)
        {
            arr->capacity = count;
            arr->data = (unsigned char*)gxmalloc(arr->capacity);
        }
        else
        {
            arr->capacity += count + VECTOR_CAPACITY;
            arr->data = (unsigned char*)gxrealloc(arr->data, arr->capacity);
        }
    }
    if (bb_getCapacity(arr) < arr->size + count)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    memcpy(arr->data + arr->size, pSource, count);
    arr->size += count;
    return 0;
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
int bb_set2(
    gxByteBuffer* arr,
    gxByteBuffer* data,
    unsigned long index,
    unsigned long count)
#else
int bb_set2(
    gxByteBuffer* arr,
    gxByteBuffer* data,
    unsigned short index,
    unsigned short count)
#endif
{
    if (data != NULL && count != 0)
    {
#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
        if (count == (unsigned long)-1)
#else
        if (count == (unsigned short)-1)
#endif
        {
            count = data->size - index;
        }
        int ret = bb_set(arr, data->data + index, count);
        if (ret == 0)
        {
            data->position += count;
        }
        return ret;
    }
    return 0;
}

int bb_addString(
    gxByteBuffer* arr,
    const char* value)
{
    if (value != NULL)
    {
        int len = (int)strlen(value);
        if (len > 0)
        {
            int ret = bb_set(arr, (const unsigned char*)value, (unsigned short)(len + 1));
            if (ret == 0)
            {
                //Add end of string, but that is not added to the length.
                arr->data[arr->size - 1] = '\0';
                --arr->size;
            }
            return ret;
        }
    }
    return 0;
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
void bb_attach(
    gxByteBuffer *arr,
    unsigned char * value,
    unsigned long count,
    unsigned long capacity)
#else
void bb_attach(
    gxByteBuffer *arr,
    unsigned char * value,
    unsigned short count,
    unsigned short capacity)
#endif
{
    arr->data = value;
#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
    arr->capacity = (0x80000000 | capacity);
#else
    arr->capacity = (unsigned short)(0x8000 | capacity);
#endif
    arr->size = count;
    arr->position = 0;
}

void bb_attachString(
    gxByteBuffer* arr,
    char* value)
{
    int len = (int)strlen(value);
    bb_set(arr, (const unsigned char*)value, (unsigned short)len);
    gxfree(value);
}

int bb_clear(
    gxByteBuffer* arr)
{
    //If byte buffer is attached.
    if (!bb_isAttached(arr))
    {
        if (arr->data != NULL)
        {
            gxfree(arr->data);
            arr->data = NULL;
        }
        arr->capacity = 0;
    }
    arr->size = 0;
    arr->position = 0;
    return 0;
}

int bb_getUInt8(
    gxByteBuffer* arr,
    unsigned char* value)
{
    if (arr->position >= arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    *value = ((unsigned char*)arr->data)[arr->position];
    ++arr->position;
    return 0;
}

int bb_getInt8(
    gxByteBuffer* arr,
    signed char* value)
{
    if (arr->position >= arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    *value = (signed char)((unsigned char*)arr->data)[arr->position];
    ++arr->position;
    return 0;
}

int bb_getUInt8ByIndex(
    gxByteBuffer* arr,
    unsigned long index,
    unsigned char* value)
{
    if (index >= arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    *value = ((unsigned char*)arr->data)[index];
    return 0;
}


int bb_getUInt16(
    gxByteBuffer* arr,
    unsigned short* value)
{

    if (arr->position + 2 > arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    *value = ((unsigned char*)arr->data)[arr->position] << 8 |
        ((unsigned char*)arr->data)[arr->position + 1];
    arr->position += 2;
    return 0;
}

int bb_getUInt32(
    gxByteBuffer* arr,
    unsigned long* value)
{

    if (arr->position + 4 > arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    *value = GETU32(arr->data + arr->position);
    arr->position += 4;
    return 0;
}

int bb_getInt16(
    gxByteBuffer* arr,
    short* value)
{

    if (arr->position + 2 > arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    *value = ((unsigned char*)arr->data)[arr->position] << 8 |
        ((unsigned char*)arr->data)[arr->position + 1];
    arr->position += 2;
    return 0;
}

int bb_getInt32(
    gxByteBuffer* arr,
    long* value)
{

    int ret = bb_getUInt32ByIndex(arr, arr->position, (unsigned long*)value);
    arr->position += 4;
    return ret;
}

int bb_getUInt32ByIndex(
    gxByteBuffer* arr,
    unsigned long index,
    unsigned long* value)
{

    if (index + 4 > arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    *value = GETU32(arr->data + index);
    return 0;
}

int bb_getInt64(
    gxByteBuffer* arr,
    long long* value)
{
    int ret = bb_getUInt64ByIndex(arr, arr->position, (unsigned long long*) value);
    if (ret == 0)
    {
        arr->position += 8;
    }
    return ret;
}

int bb_getUInt64(
    gxByteBuffer* arr,
    unsigned long long* value)
{
    int ret = bb_getUInt64ByIndex(arr, arr->position, value);
    if (ret == 0)
    {
        arr->position += 8;
    }
    return ret;
}

int bb_getUInt64ByIndex(
    gxByteBuffer* arr,
    unsigned long index,
    unsigned long long* value)
{
    if (index + 8 > arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    *value = GETU32(arr->data + index);
    //Some 32 bit microcontrollers can't handle *value <<= 32;
    //For this reason value is sifted on two parts.
    *value <<= 16;
    *value <<= 16;
    *value |= GETU32(arr->data + index + 4);
    return 0;
}

int bb_getUInt128ByIndex(
    gxByteBuffer* arr,
    unsigned long index,
    unsigned char* value)
{
    int ret = bb_getUInt32ByIndex(arr, index, (unsigned long*)value);
    if (ret == 0)
    {
        ret = bb_getUInt32ByIndex(arr, index + 4, (unsigned long*)value + 1);
        if (ret == 0)
        {
            ret = bb_getUInt32ByIndex(arr, index + 8, (unsigned long*)value + 2);
            if (ret == 0)
            {
                ret = bb_getUInt32ByIndex(arr, index + 12, (unsigned long*)value + 3);
            }
        }
    }
    return ret;
}

#ifndef GX_DLMS_MICROCONTROLLER
int bb_getFloat(
    gxByteBuffer* arr,
    float* value)
{
    typedef union
    {
        float value;
        char b[sizeof(float)];
    } HELPER;
    HELPER tmp;
    if (arr->position + 4 > arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    tmp.b[3] = arr->data[arr->position];
    tmp.b[2] = arr->data[arr->position + 1];
    tmp.b[1] = arr->data[arr->position + 2];
    tmp.b[0] = arr->data[arr->position + 3];
    *value = tmp.value;
    arr->position += 4;
    return 0;
}

int bb_getDouble(
    gxByteBuffer* arr,
    double* value)
{
    typedef union
    {
        double value;
        char b[sizeof(double)];
    } HELPER;
    HELPER tmp;
    if (arr->position + 8 > arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    tmp.b[7] = arr->data[arr->position];
    tmp.b[6] = arr->data[arr->position + 1];
    tmp.b[5] = arr->data[arr->position + 2];
    tmp.b[4] = arr->data[arr->position + 3];
    tmp.b[3] = arr->data[arr->position + 4];
    tmp.b[2] = arr->data[arr->position + 5];
    tmp.b[1] = arr->data[arr->position + 6];
    tmp.b[0] = arr->data[arr->position + 7];
    *value = tmp.value;
    arr->position += 8;
    return 0;
}
#endif //GX_DLMS_MICROCONTROLLER

int bb_getUInt16ByIndex(
    gxByteBuffer* arr,
    unsigned long index,
    unsigned short* value)
{
    if (index + 2 > arr->size)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    *value = ((unsigned char*)arr->data)[index] << 8 |
        ((unsigned char*)arr->data)[index + 1];
    return 0;
}

int bb_addHexString(
    gxByteBuffer* arr,
    const char* str)
{
    unsigned char count;
    int ret;
    unsigned char* buffer = NULL;
    ret = hlp_hexToBytes(str, &buffer, &count);
    if (ret != 0)
    {
        return ret;
    }
    if (buffer != NULL)
    {
        bb_set(arr, buffer, count);
        gxfree(buffer);
    }
    return 0;
}

#ifndef GX_DLMS_MICROCONTROLLER
char* bb_toString(
    gxByteBuffer* arr)
{
    char* buff = (char*)gxmalloc(arr->size + 1);
    memcpy(buff, arr->data, arr->size);
    *(buff + arr->size) = 0;
    return buff;
}

char* bb_toHexString(
    gxByteBuffer* arr)
{
    char* buff = hlp_bytesToHex(arr->data, arr->size);
    return buff;
}

void bb_addIntAsString(
    gxByteBuffer* bb,
    int value)
{
    char str[20];
    hlp_intToString(str, 20, value, 1);
    bb_addString(bb, str);
}

void bb_addDoubleAsString(
    gxByteBuffer* bb,
    double value)
{
    char buff[20];
    //Show as integer value if there is no fractal part.
    if (value - (long)value == 0)
    {
        bb_addIntAsString(bb, (int)value);
    }
    else
    {
#if _MSC_VER > 1000
        sprintf_s(buff, 20, "%lf", value);
#else
        sprintf(buff, "%lf", value);
#endif
        bb_addString(bb, buff);
    }
}
#endif //GX_DLMS_MICROCONTROLLER

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
int bb_subArray(
    gxByteBuffer* bb,
    unsigned long index,
    unsigned long count,
    gxByteBuffer* target)
#else
int bb_subArray(
    gxByteBuffer* bb,
    unsigned short index,
    unsigned short count,
    gxByteBuffer* target)
#endif
{
    bb_clear(target);
    bb_set2(target, bb, index, count);
    return 0;
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
int bb_insert(const unsigned char* src,
    unsigned long count,
    gxByteBuffer* target,
    unsigned long index)
#else
int bb_insert(const unsigned char* src,
    unsigned short count,
    gxByteBuffer* target,
    unsigned short index)
#endif
{
    int ret;
    if (target->size == 0)
    {
        ret = bb_set(target, src, count);
    }
    else
    {
        if ((ret = bb_capacity(target, target->size + count)) == 0 &&
            (ret = bb_move(target, index, index + count, target->size - index)) == 0)
        {
            //Do not use memcpy here!
            memmove(target->data + index, src + index, count);
        }
    }
    return ret;
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
int bb_move(
    gxByteBuffer* bb,
    unsigned long srcPos,
    unsigned long destPos,
    unsigned long count)
#else
int bb_move(
    gxByteBuffer* bb,
    unsigned short srcPos,
    unsigned short destPos,
    unsigned short count)
#endif
{
    //If items are removed.
    if (srcPos > destPos)
    {
        if (bb->size < destPos + count)
        {
            return DLMS_ERROR_CODE_INVALID_PARAMETER;
        }
    }
    else
    {
        //Append data.
        if (bb_getCapacity(bb) < count + destPos - srcPos)
        {
            int ret;
            if (bb_isAttached(bb))
            {
                return DLMS_ERROR_CODE_INVALID_PARAMETER;
            }
            if ((ret = bb_capacity(bb, count + destPos - srcPos)) != 0)
            {
                return ret;
            }
        }
    }
    if (count != 0)
    {
        //Do not use memcpy here!
        memmove(bb->data + destPos, bb->data + srcPos, count);
        if (destPos < srcPos)
        {
            //If data is moved to the begin.
            bb->size = (destPos + count);
        }
        else
        {
            //If data is append.
            bb->size = (destPos - srcPos + count);
        }
        if (bb->position > bb->size)
        {
            bb->position = bb->size;
        }
    }
    return DLMS_ERROR_CODE_OK;
}

int bb_trim(
    gxByteBuffer* bb)
{
    int ret;
    if (bb->size == bb->position)
    {
        bb->size = 0;
    }
    else
    {
        if ((ret = bb_move(bb, bb->position, 0, bb->size - bb->position)) != 0)
        {
            return ret;
        }
    }
    bb->position = 0;
    return DLMS_ERROR_CODE_OK;
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
unsigned char bb_compare(
    gxByteBuffer* bb,
    unsigned char* buff,
    unsigned long length)
#else
unsigned char bb_compare(
    gxByteBuffer* bb,
    unsigned char* buff,
    unsigned short length)
#endif

{
    unsigned char equal;
    if (bb->size - bb->position < length)
    {
        return 0;
    }
    equal = memcmp(bb->data + bb->position, buff, length) == 0;
    if (equal)
    {
        bb->position += length;
    }
    return equal;
}

#if !defined(GX_DLMS_MICROCONTROLLER) && (defined(_WIN32) || defined(_WIN64) || defined(__linux__))
int bb_get(
    gxByteBuffer* bb,
    unsigned char* value,
    unsigned long count)
#else
int bb_get(
    gxByteBuffer* bb,
    unsigned char* value,
    unsigned short count)
#endif
{
    if (bb == NULL || value == NULL || bb->size - bb->position < count)
    {
        return DLMS_ERROR_CODE_OUTOFMEMORY;
    }
    memcpy(value, bb->data + bb->position, count);
    bb->position += count;
    return 0;
}

unsigned long bb_indexOf(
    gxByteBuffer* bb,
    char ch)
{
    unsigned long pos;
    if (bb == NULL)
    {
        return (unsigned long)-1;
    }
    for (pos = bb->position; pos < bb->size; ++pos)
    {
        if (bb->data[pos] == ch)
        {
            return pos;
        }
    }
    return (unsigned long)-1;
}

#ifndef GX_DLMS_MICROCONTROLLER
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
void bb_print(gxByteBuffer* bb)
{
    const char hexArray[] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
    unsigned long pos;
    char hexChars[4];
    hexChars[2] = ' ';
    hexChars[3] = '\0';
    for (pos = 0; pos != bb->size; ++pos)
    {
        hexChars[0] = hexArray[bb->data[pos] >> 4];
        hexChars[1] = hexArray[bb->data[pos] & 0x0F];
        printf(hexChars);
    }
}
#endif //defined(_WIN32) || defined(_WIN64) || defined(__linux__)
#endif //GX_DLMS_MICROCONTROLLER
