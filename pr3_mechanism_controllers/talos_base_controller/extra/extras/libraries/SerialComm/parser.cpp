#include <string.h>
//#include <stdio.h>
#include "Arduino.h"
#include "parser.h"

int length(const char* input)
{
	int length = 0;
	char* c = (char*)input;
	while (*c != '\0')
	{
		length++;
		c++;
	}
	return length;
}

void parse_twist(const char* msg, double* buf)
{
	int len = length(msg);
	char tmpbuf[len+1];
	strcpy( tmpbuf, msg );
	
	char *str1, *str2, *token, *subtoken;
	char *saveptr1, *saveptr2;
	int j;
	for (j = 0, str1 = tmpbuf; ; j++, str1 = NULL) {
		token = strtok_r(str1, " ,", &saveptr1);
		if (token == NULL)
			break;
		buf[j] = (float)atof(token);
	}
}


