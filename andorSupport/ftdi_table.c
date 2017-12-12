/*
	This is a simple method to enable users to configure their own table of 
	VID and PIDs.
	Just overwrite entries in the table or add/subract as many as you want.
	There may be a performance drop when using FT_Open, FT_OpenEx or 
	FT_ListDevices when using this method. Therefore it is advised to avoid
	excessive used of these functions.

  Compile with:
  $ gcc -fpic -shared -Wl,-soname,libd2xx_table.so -o libd2xx_table.so ftdi_table.c
  
  This is Andor Shamrock 500i:
  Bus 001 Device 073: ID 0403:d493 Future Technology Devices International, Ltd 

*/


#define SIZE(x)	(sizeof(x)/sizeof(x[0]))

static int id_table[][2] = {
	{ 0x0403, 0x6010 },			/* 2232C */
	{ 0x0403, 0x6001 },			/* AM or BM */
	{ 0x0403, 0x6006 },			/* Direct Driver PID */
	{ 0x0403, 0xD493 }			/* Andor Shamrock 500i */
};

/*
	int lib_check_device(int vendor, int product)
	
	Description:	Check the VID and PID against our table	
	Arguments:		vendor (device to check VID), product (device to check PID)
	Return:			0 if no match, 1 if match
*/
int lib_check_device(int vendor, int product)
{
	int i;

	for(i = 0; i < SIZE(id_table); i++) {
		if ((id_table[i][0] == vendor) && (id_table[i][1] == product)) {
			return 1;
		}
	}
	return 0; /* no match */
}


