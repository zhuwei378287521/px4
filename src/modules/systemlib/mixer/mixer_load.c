/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mixer_load.c
 *加载mixer参数
 * Programmable multi-channel mixer library.
 */

#include <px4_config.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <systemlib/err.h>

#include "mixer_load.h"
//加载文件，所有的mixer都写成一个文件，通过存取sd卡，用指令加载到romFS中
/**
 * input：fname文件名，buf读取文件内容结果，maxlen为buf最大长度
 * 该函数会剔除符合下面任何一条的行：
1.行长度小于2个字符的行
2.行的首字符不是大写字母的行
3.第二个字符不是':'的行
剔除这些非法内容的数据后，剩余的全部为格式化的内容，会被全部存入buf缓冲区中。
所以这要求在写mix文件时要遵循mix和格式。
这些格式化的mix内容被读取缓冲区后，就会通过函数
int ret = ioctl(dev, MIXERIOCLOADBUF, (unsigned long)buf);
来交给具体的设备处理。
 */
int load_mixer_file(const char *fname, char *buf, unsigned maxlen)
{
	FILE		*fp;
	char		line[120];

	/* open the mixer definition file */
	fp = fopen(fname, "r");//打开目标文件

	if (fp == NULL) {
		warnx("file not found");
		return -1;
	}

	/* read valid lines from the file into a buffer */
	buf[0] = '\0';

	for (;;) {//死循环

		/* get a line, bail on error/EOF */
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == NULL) {//获取一整行
			break;
		}

		/* if the line doesn't look like a mixer definition line, skip it */
		if ((strlen(line) < 2) || !isupper(line[0]) || (line[1] != ':')) {
			continue;//去除小于2个字符，开头不是大写，第二字符不是：号的行
		}

		/* compact whitespace in the buffer */
		char *t, *f;

		for (f = line; *f != '\0'; f++) {
			/* scan for space characters *///去除空格
			if (*f == ' ') {
				/* look for additional spaces */
				t = f + 1;

				while (*t == ' ') {
					t++;
				}

				if (*t == '\0') {
					/* strip trailing whitespace */
					*f = '\0';//加入末尾

				} else if (t > (f + 1)) {
					memmove(f + 1, t, strlen(t) + 1);
				}
			}
		}

		/* if the line is too long to fit in the buffer, bail */
		if ((strlen(line) + strlen(buf) + 1) >= maxlen) {
			warnx("line too long");
			fclose(fp);
			return -1;
		}//太长，抛弃

		/* add the line to the buffer */
		strcat(buf, line);//复制信息到buf
	}

	fclose(fp);
	return 0;
}

