#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#define R(x)	(unsigned char)(32.0*x/256.0)
#define G(x)	(unsigned char)(64.0*x/256.0)
#define B(x)	(unsigned char)(32.0*x/256.0)

int main(int argc, char *argv[])
{
	int fdi, j, ret;
	unsigned char buf[4096];
	unsigned long l;
	FILE *fdo;

	if (argc < 2)
	{
		printf("bin2hex sourcefile dstfile\n");
		exit(1);
	}

	fdi = open(argv[1], O_RDONLY);
	if(fdi < 0)
	{
		printf("fdi err\n");
		exit(1);
	}

	l = lseek(fdi, 0, SEEK_END);
	lseek(fdi, 0, SEEK_SET);

	fdo = fopen(argv[2], "w");
	if(fdo < 0)
	{
		printf("fdo err\n");
		exit(1);
	}

	fprintf(fdo, "/* Auto generated hex data from file:\n * %s\n */\n", argv[1]);
	fprintf(fdo, "const unsigned char imapfb_oem_logo_data[0x%lx] = {\n", l * 2 / 3);
	while(1)
	{
		if((ret = read(fdi, buf, 24)))
		{
			fprintf(fdo, "\t");
			for (j = 0; j < ret; j++)
				if(!(j % 3)) {
					fprintf(fdo, "0x%02x, ", (unsigned char)(G(buf[j + 1]) << 5) | (B(buf[j + 2])));
					fprintf(fdo, "0x%02x, ", (unsigned char)(R(buf[j]) << 3) | (G(buf[j + 1]) >> 3));
				}
			fprintf(fdo, "\n");

			if(ret % 3)
			  printf("warning: the source may be not 888.\n");
		}
		else
		  break;
	}

	fprintf(fdo, "};");

	fclose(fdo);
	close(fdi);
	return 0;
}
