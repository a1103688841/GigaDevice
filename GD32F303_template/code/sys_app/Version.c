
/*
 * Version.c
 *
 * Copyright (C) 2012 Alsor Chen
 *
 * This file is S32F3 version api
 *
 * Change Activity:
 *
 * 2012-04-05 by Alsor Chen
 *   First create.
 *   Put version information into this file, Set this file to always builded to update the time information
 *   2012-10-22 by Alsor Chen
 *   Change the Firmware version into xx.xx.xx format.
 */

#include "version.h"

#include <stdio.h>
#include <string.h>

/************************************************************************/
/*      Constants                                                       */
/************************************************************************/
#define VersionLen 16

/************************************************************************/
/*      Type Definitions                                                */
/************************************************************************/
struct project_version_s{
    unsigned char VersionValid;
    char Auther[VersionLen];
    char System[VersionLen];
    char CompanyName[VersionLen];
    char ProductName[VersionLen];
    char ProductModel[VersionLen];
    char VersionNum[VersionLen];
    char VersionTime[VersionLen];
    char Feature[VersionLen];
};


/************************************************************************/
/*      Local Prototypes                                                */
/************************************************************************/

/************************************************************************/
/*      Global Variables                                                */
/************************************************************************/
#ifdef _80C51_MCU
code struct project_version_s OwnVersion = {
#else
const struct project_version_s OwnVersion = {
#endif
    1,
    "Inker.Dong",
    "DBG_TIM",
    "利利普",
    "康莱德万用表",
    "VC891",
    "1.01",
    "2021/10/27",

    /*1:Debug or Release Version
     *  D=Debug Version
     *  R=Release Version */
#ifdef _GLOBAL_DEBUG_ENABLE
    "D"
#else
    "R"
#endif

};

/************************************************************************/
/*      Local   Variables                                               */
/************************************************************************/
const unsigned char BuildedDateStr[] = __DATE__;
const unsigned char BuildedTimeStr[] = __TIME__;

/************************************************************************/
/*      Application Interface                                           */
/************************************************************************/
char *GetAuther(void)
{
    return (char *)OwnVersion.Auther;
}
char *GetSystem(void)
{
    return (char *)OwnVersion.System;
}
char *GetCompany(void)
{
    return (char *)OwnVersion.CompanyName;
}
char *GetProName(void)
{
    return (char *)OwnVersion.ProductName;
}
char *GetProModel(void)
{
    return (char *)OwnVersion.ProductModel;
}
char *GetVersionNum(void)
{
    return (char *)OwnVersion.VersionNum;
}
char *GetVersionTime(void)
{
    return (char *)OwnVersion.VersionTime;
}
char *GetBoardFeature(void)
{
    return (char *)OwnVersion.Feature;
}
char *GetBuildedDate(void)
{
    return (char *)BuildedDateStr;
}
char *GetBuildedTime(void)
{
    return (char *)BuildedTimeStr;
}
/************************************************************************/
/*     dbg_cmd Interface                                                */
/************************************************************************/
#include "dbg_cmd.h"
#ifdef DBG_CMD_EN
void dbg_cmd_version(void)
{
    DBG_CMD_PRN(" System:%s\r\n", GetSystem());
    DBG_CMD_PRN(" Auther:%s\r\n", GetAuther());
    DBG_CMD_PRN("Company:%s\r\n", GetCompany());
    DBG_CMD_PRN("Product:%s,%s\r\n", GetProName(), GetProModel());
    DBG_CMD_PRN("Version:%s,%s\r\n", GetVersionNum(), GetBoardFeature());
    DBG_CMD_PRN("   Time:%s\r\n",    GetVersionTime());
    DBG_CMD_PRN("Builded:%s,%s\r\n", GetBuildedDate(), GetBuildedTime());
}
static bool dbg_cmd_func(void)
{
    if (dbg_cmd_exec("help", "", "")) {
        DBG_CMD_PRN(".Version\r\n");
        return false;
    }
    if (dbg_cmd_exec(".Version", "", "")) {
        dbg_cmd_print_msg_en();
    }
    if (dbg_cmd_exec("vermsg", "", "")) {
        dbg_cmd_version();
        return true;
    }
    if (dbg_cmd_exec("VerLog", "", "")) {
        DBG_CMD_PRN("200730_V1.01:Add Printf Log Message!\r\n");
        return true;
    }
    return false;
}
#endif
/************************************************************************/
/*     Application Interface                                            */
/************************************************************************/
void version_init(void)
{
#ifdef DBG_CMD_EN
    dbg_cmd_add_list((long)dbg_cmd_func);
#endif // DBG_CMD_EN
}

