/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "flashlight.h"

#if defined(mt6757)
	#if defined(evb6757p_dm_64) || defined(k57pv1_dm_64) || \
	defined(k57pv1_64_baymo) || defined(k57pv1_dm_64_bif) || \
	defined(k57pv1_dm_64_baymo)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-s2mu005", 0, 0}, //  <== main + CT0
		{1, 0, 0, "flashlights-s2mu005", 1, 0}, //  <== main + CT1 (not exist)
		{0, 1, 0, "flashlights-none", -1, 0},   //  <== sub + CT0
		{1, 1, 0, "flashlights-none", -1, 0},  //  <== sub + CT1 (not exist)
		{0, 0, 1, "flashlights-none", -1, 0},
		{0, 1, 1, "flashlights-none", -1, 0},
		{1, 0, 1, "flashlights-none", -1, 0},
		{1, 1, 1, "flashlights-none", -1, 0},
	};
	#elif defined(titan6757_phone_n)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-s2mu005", 0, 0}, //	<== main + CT0
		{1, 0, 0, "flashlights-s2mu005", 1, 0}, //	<== main + CT1 (not exist)
		{0, 1, 0, "flashlights-none", -1, 0},	//	<== sub + CT0
		{1, 1, 0, "flashlights-none", -1, 0},  //  <== sub + CT1 (not exist)
		{0, 0, 1, "flashlights-none", -1, 0},
		{0, 1, 1, "flashlights-none", -1, 0},
		{1, 0, 1, "flashlights-none", -1, 0},
		{1, 1, 1, "flashlights-none", -1, 0},
	};
	#elif defined(titan6757_jade_n)
		#if defined(CONFIG_MTK_FLASHLIGHT_SM5705)
			const struct flashlight_device_id flashlight_id[] = {
				/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
				{0, 0, 0, "flashlights-sm5705", 0, 0},
				{1, 0, 0, "flashlights-sm5705", 1, 0},		
				{0, 1, 0, "flashlights-none", -1, 0},
				{1, 1, 0, "flashlights-none", -1, 0},
				{0, 0, 1, "flashlights-none", -1, 0},
				{0, 1, 1, "flashlights-none", -1, 0},
				{1, 0, 1, "flashlights-none", -1, 0},
				{1, 1, 1, "flashlights-none", -1, 0},
			};
		#else
			const struct flashlight_device_id flashlight_id[] = {
				/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
				{0, 0, 0, "flashlights-s2mu005", 0, 0}, //	<== main + CT0
				{1, 0, 0, "flashlights-s2mu005", 1, 0}, //	<== main + CT1 (not exist)
				{0, 1, 0, "flashlights-none", -1, 0},	//	<== sub + CT0
				{1, 1, 0, "flashlights-none", -1, 0},  //  <== sub + CT1 (not exist)
				{0, 0, 1, "flashlights-none", -1, 0},
				{0, 1, 1, "flashlights-none", -1, 0},
				{1, 0, 1, "flashlights-none", -1, 0},
				{1, 1, 1, "flashlights-none", -1, 0},
			};
		#endif
	#elif defined(titan6757_c10_n)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-sm5705", 0, 0},
		{0, 1, 0, "flashlights-sm5705", 1, 0},		
		{0, 1, 0, "flashlights-none", -1, 0},
		{1, 1, 0, "flashlights-none", -1, 0},
		{0, 0, 1, "flashlights-none", -1, 0},
		{0, 1, 1, "flashlights-none", -1, 0},
		{1, 0, 1, "flashlights-none", -1, 0},
		{1, 1, 1, "flashlights-none", -1, 0},
	};
	#elif defined(titan6757_jade_n)
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
		{0, 0, 0, "flashlights-s2mu005", 0, 0},
		{0, 1, 0, "flashlights-none", -1, 0},
		{1, 0, 0, "flashlights-s2mu005", 1, 0},
		{1, 1, 0, "flashlights-none", -1, 0},
		{0, 0, 1, "flashlights-none", -1, 0},
		{0, 1, 1, "flashlights-none", -1, 0},
		{1, 0, 1, "flashlights-none", -1, 0},
		{1, 1, 1, "flashlights-none", -1, 0},
	};
	#else
	const struct flashlight_device_id flashlight_id[] = {
		/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
        {0, 0, 0, "flashlights-s2mu005", 0, 0}, //  <== main + CT0
        {1, 0, 0, "flashlights-s2mu005", 1, 0}, //  <== main + CT1 (not exist)
        {0, 1, 0, "flashlights-none", -1, 0},   //  <== sub + CT0
        {1, 1, 0, "flashlights-none", -1, 0},  //  <== sub + CT1 (not exist)
		{0, 0, 1, "flashlights-none", -1, 0},
		{0, 1, 1, "flashlights-none", -1, 0},
		{1, 0, 1, "flashlights-none", -1, 0},
		{1, 1, 1, "flashlights-none", -1, 0},
	};
	#endif
#elif defined(mt6799)
const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-mt6336", 0, 0},
	{0, 1, 0, "flashlights-mt6336", 1, 0},
	{1, 0, 0, "flashlights-none", -1, 0},
	{1, 1, 0, "flashlights-none", -1, 0},
	{0, 0, 1, "flashlights-none", -1, 0},
	{0, 1, 1, "flashlights-none", -1, 0},
	{1, 0, 1, "flashlights-none", -1, 0},
	{1, 1, 1, "flashlights-none", -1, 0},
};
#else
const struct flashlight_device_id flashlight_id[] = {
	/* {TYPE, CT, PART, "NAME", CHANNEL, DECOUPLE} */
	{0, 0, 0, "flashlights-none", -1, 0},
	{0, 1, 0, "flashlights-none", -1, 0},
	{1, 0, 0, "flashlights-none", -1, 0},
	{1, 1, 0, "flashlights-none", -1, 0},
	{0, 0, 1, "flashlights-none", -1, 0},
	{0, 1, 1, "flashlights-none", -1, 0},
	{1, 0, 1, "flashlights-none", -1, 0},
	{1, 1, 1, "flashlights-none", -1, 0},
};
#endif

