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
#include "../include/dlmssettings.h"
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)
#include <assert.h>
#include <stdio.h>
#endif

// Server sender frame sequence starting number.
static const unsigned char SERVER_START_SENDER_FRAME_SEQUENCE = 0x1E;
// Server receiver frame sequence starting number.
static const unsigned char SERVER_START_RECEIVER_FRAME_SEQUENCE = 0xFE;
// Client sender frame sequence starting number.
static const unsigned char CLIENT_START_SENDER_FRAME_SEQUENCE = 0xFE;
// Client receiver frame sequence starting number.
static const unsigned char CLIENT_START_RCEIVER_FRAME_SEQUENCE = 0xE;

//Initialize server.
void svr_init(
    dlmsServerSettings* settings,
    unsigned char useLogicalNameReferencing,
    DLMS_INTERFACE_TYPE interfaceType,
    unsigned short frameSize,
    unsigned short pduSize,
    unsigned char* frameBuffer,
    unsigned short frameBufferSize,
    unsigned char* pduBuffer,
    unsigned short pduBufferSize)
{
    cl_init(&settings->base, useLogicalNameReferencing, 0, 0, DLMS_AUTHENTICATION_NONE, NULL, interfaceType);
	//cl_init(&settings->base, useLogicalNameReferencing, 0, 0, DLMS_AUTHENTICATION_LOW, "devsys", interfaceType);
    settings->base.proposedConformance |= DLMS_CONFORMANCE_GENERAL_PROTECTION;
    bb_attach(&settings->receivedData, frameBuffer, 0, frameBufferSize);
    reply_init(&settings->info);
    bb_attach(&settings->info.data, pduBuffer, 0, pduBufferSize);
    settings->base.maxInfoRX = settings->base.maxInfoTX = frameSize;
    settings->base.maxServerPDUSize = pduSize;
    settings->base.server = 1;
    settings->initialized = 0;
    trans_init(&settings->transaction);
#ifndef DLMS_IGNORE_TCP_UDP_SETUP
    settings->wrapper = NULL;
#endif // DLMS_IGNORE_TCP_UDP_SETUP
#ifndef DLMS_IGNORE_IEC_HDLC_SETUP
    settings->hdlc = NULL;
#endif //DLMS_IGNORE_IEC_HDLC_SETUP

    settings->dataReceived = 0;
    settings->frameReceived = 0;
    resetFrameSequence(&settings->base);
}

//Initialize client.
void cl_init(
    dlmsSettings* settings,
    unsigned char useLogicalNameReferencing,
    int clientAddress,
    int serverAddress,
    DLMS_AUTHENTICATION authentication,
    const char* password,
    DLMS_INTERFACE_TYPE interfaceType)
{
    settings->qualityOfService = 0;
    settings->protocolVersion = NULL;
    settings->preEstablishedSystemTitle = NULL;
    settings->blockIndex = 1;
    settings->clientAddress = (unsigned short)clientAddress;
    settings->serverAddress = (unsigned short)serverAddress;
    settings->dlmsVersionNumber = 6;
    settings->useLogicalNameReferencing = useLogicalNameReferencing;
    settings->interfaceType = interfaceType;
    settings->authentication = authentication;
    bb_init(&settings->password);
    bb_addString(&settings->password, password);
    bb_init(&settings->sourceSystemTitle);
    bb_init(&settings->kek);
    settings->maxServerPDUSize = 1024;
    settings->maxPduSize = 0xFFFF;
    settings->position = settings->index = settings->count = 0;
    settings->server = 0;
    if (useLogicalNameReferencing)
    {
        settings->proposedConformance = (DLMS_CONFORMANCE)(DLMS_CONFORMANCE_BLOCK_TRANSFER_WITH_ACTION |
            DLMS_CONFORMANCE_BLOCK_TRANSFER_WITH_SET_OR_WRITE |
            DLMS_CONFORMANCE_BLOCK_TRANSFER_WITH_GET_OR_READ |
            DLMS_CONFORMANCE_SET | DLMS_CONFORMANCE_SELECTIVE_ACCESS |
            DLMS_CONFORMANCE_ACTION | DLMS_CONFORMANCE_MULTIPLE_REFERENCES |
            DLMS_CONFORMANCE_GET);
    }
    else
    {
        settings->proposedConformance = (DLMS_CONFORMANCE)(DLMS_CONFORMANCE_INFORMATION_REPORT |
            DLMS_CONFORMANCE_READ | DLMS_CONFORMANCE_UN_CONFIRMED_WRITE |
            DLMS_CONFORMANCE_WRITE | DLMS_CONFORMANCE_PARAMETERIZED_ACCESS |
            DLMS_CONFORMANCE_MULTIPLE_REFERENCES);
    }
    settings->longInvokeID = 0;
    settings->maxInfoTX = settings->maxInfoRX = 0x80;
    settings->windowSizeTX = settings->windowSizeRX = 1;
    settings->connected = DLMS_CONNECTION_STATE_NONE;
    oa_init(&settings->objects);
    settings->connected = DLMS_CONNECTION_STATE_NONE;
    settings->customChallenges = 0;
    settings->invokeID = 1;
    bb_init(&settings->ctoSChallenge);
    bb_init(&settings->stoCChallenge);
    settings->priority = DLMS_PRIORITY_HIGH;
    settings->serviceClass = DLMS_SERVICE_CLASS_UN_CONFIRMED;
#ifndef DLMS_IGNORE_HIGH_GMAC
    cip_init(&settings->cipher);
#endif //DLMS_IGNORE_HIGH_GMAC
    settings->userId = -1;
    resetFrameSequence(settings);
}

void cl_clear(
    dlmsSettings* settings)
{
    if (settings->protocolVersion != NULL)
    {
        gxfree(settings->protocolVersion);
        settings->protocolVersion = NULL;
    }
    if (settings->preEstablishedSystemTitle != NULL)
    {
        bb_clear(settings->preEstablishedSystemTitle);
        gxfree(settings->preEstablishedSystemTitle);
        settings->preEstablishedSystemTitle = NULL;
    }
    bb_clear(&settings->password);
    bb_clear(&settings->sourceSystemTitle);
    bb_clear(&settings->kek);
    oa_clear(&settings->objects);
    settings->connected = DLMS_CONNECTION_STATE_NONE;
    settings->customChallenges = 0;
    settings->invokeID = 1;
    bb_clear(&settings->ctoSChallenge);
    bb_clear(&settings->stoCChallenge);
    settings->priority = DLMS_PRIORITY_HIGH;
    settings->serviceClass = DLMS_SERVICE_CLASS_UN_CONFIRMED;
#ifndef DLMS_IGNORE_HIGH_GMAC
    cip_clear(&settings->cipher);
#endif //DLMS_IGNORE_HIGH_GMAC
    settings->maxPduSize = 0xFFFF;
    settings->position = settings->index = settings->count = 0;
    settings->userId = -1;
}

void svr_clear(
    dlmsServerSettings* settings)
{
    cl_clear(&settings->base);
}

void resetBlockIndex(
    dlmsSettings* settings)
{
    settings->blockIndex = 1;
}

void resetFrameSequence(
    dlmsSettings* settings)
{
    if (settings->server)
    {
        settings->senderFrame = SERVER_START_SENDER_FRAME_SEQUENCE;
        settings->receiverFrame = SERVER_START_RECEIVER_FRAME_SEQUENCE;
    }
    else
    {
        settings->senderFrame = CLIENT_START_SENDER_FRAME_SEQUENCE;
        settings->receiverFrame = CLIENT_START_RCEIVER_FRAME_SEQUENCE;
    }
}

unsigned char increaseReceiverSequence(
    unsigned char value)
{
    return ((value + 0x20) | 0x10 | (value & 0xE));
}

// Increase sender sequence.
//
// @param value
//            Frame value.
// Increased sender frame sequence.
unsigned char increaseSendSequence(
    unsigned char value)
{
    return (unsigned char)((value & 0xF0) | ((value + 0x2) & 0xE));
}

unsigned char checkFrame(
    dlmsSettings* settings,
    unsigned char frame)
{
    //If notify
    if (frame == 0x13)
    {
        return 1;
    }
    // If U frame.
    if ((frame & 0x3) == 3)
    {
        if (frame == 0x73 || frame == 0x93)
        {
            resetFrameSequence(settings);
        }
        return 1;
    }
    // If S -frame
    if ((frame & 0x3) == 1)
    {
        unsigned char ch = increaseReceiverSequence(settings->receiverFrame);
        if ((frame & 0xE0) != (ch & 0xE0))
        {
            return 0;
        }
        settings->receiverFrame = increaseReceiverSequence(settings->receiverFrame);
        return 1;
    }
    //Handle I-frame.
    unsigned char expected;
    if ((settings->senderFrame & 0x1) == 0)
    {
        expected = increaseReceiverSequence(increaseSendSequence(settings->receiverFrame));
        if (frame == expected)
        {
            settings->receiverFrame = frame;
            return 1;
        }
    }
    //If answer for RR.
    else
    {
        expected = increaseSendSequence(settings->receiverFrame);
        if (frame == expected)
        {
            settings->receiverFrame = frame;
            return 1;
        }
    }
    //Pre-established connections needs this.
    if ((!settings->server && settings->receiverFrame == SERVER_START_RECEIVER_FRAME_SEQUENCE) ||
        (settings->server && settings->receiverFrame == CLIENT_START_RCEIVER_FRAME_SEQUENCE))
    {
        settings->receiverFrame = frame;
        return 1;
    }
#if defined(_WIN32) || defined(_WIN64) || defined(__linux__)//If Windows or Linux
    printf("Invalid frame %X. Expected %X.\r\n", frame, expected);
#endif
    return 0;
}

unsigned char getNextSend(
    dlmsSettings* settings, unsigned char first)
{
    if (first)
    {
        settings->senderFrame = increaseReceiverSequence(increaseSendSequence((unsigned char)settings->senderFrame));
    }
    else
    {
        settings->senderFrame = increaseSendSequence((unsigned char)settings->senderFrame);
    }
    return (unsigned char)settings->senderFrame;
}

unsigned char getReceiverReady(
    dlmsSettings* settings)
{
    settings->senderFrame = increaseReceiverSequence((unsigned char)(settings->senderFrame | 1));
    return (unsigned char)(settings->senderFrame & 0xF1);
}

unsigned char getKeepAlive(
    dlmsSettings* settings)
{
    settings->senderFrame = (unsigned char)(settings->senderFrame | 1);
    return (unsigned char)(settings->senderFrame & 0xF1);
}

#ifndef DLMS_IGNORE_HIGH_GMAC
unsigned char isCiphered(
    ciphering* cipher)
{
    return cipher->security != DLMS_SECURITY_NONE;
}
#endif //DLMS_IGNORE_HIGH_GMAC

void trans_init(gxLongTransaction* trans)
{
    trans->command = DLMS_COMMAND_NONE;
    bb_init(&trans->data);
    vec_init(&trans->targets);
}

void trans_clear(gxLongTransaction* trans)
{
    trans->command = DLMS_COMMAND_NONE;
    bb_clear(&trans->data);
    vec_clear(&trans->targets);
}

void updateInvokeId(
    dlmsServerSettings* settings,
    unsigned char value)
{
    if ((value & 0x80) != 0) {
        settings->base.priority = DLMS_PRIORITY_HIGH;
    }
    else {
        settings->base.priority = DLMS_PRIORITY_NORMAL;
    }
    if ((value & 0x40) != 0) {
        settings->base.serviceClass = DLMS_SERVICE_CLASS_CONFIRMED;
    }
    else {
        settings->base.serviceClass = DLMS_SERVICE_CLASS_UN_CONFIRMED;
    }
    settings->base.invokeID = (unsigned char)(value & 0xF);
}
