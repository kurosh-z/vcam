#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/hidraw.h>
#include <libudev.h>

#include <errno.h>
#include <linux/videodev2.h>

#define CAMERA_CONTROL_SEE3CAM_20CUG    0xA1
#define SET_SUCCESS                     0x01
#define SET_FAILURE                     0x00

#define SET_STREAM_MODE_20CUG                 0x02
#define GET_STREAM_MODE_20CUG                 0x01


#define SET_FLASH_MODE_20CUG                   0x04
#define GET_FLASH_MODE_20CUG                   0x03

#define SET_FLIP_CONTROLS_20CUG                0x06
#define GET_FLIP_CONTROLS_20CUG                0x05

#define SET_DEFAULT_20CUG                      0xFF

#define READ_FIRMWARE_VERSION_20CUG            0x40

#define READ_UNIQUE_ID_20CUG                   0x41




#define SUCCESS				0
#define FAILURE 			-1
#define ENABLE				1
#define DISABLE				0

#define ECON_VID				"2560"
#define CAMERA_PID				"c124"	// PID of See3CAM_20CUG


#define SET_FAIL			0x00
#define SET_SUCCESS			0x01
#define GET_FAIL			0x00
#define GET_SUCCESS			0x01
#define BUFFER_LENGTH			65

#define CLEAR(x) memset(&(x), 0, sizeof(x))

char* hid_device;
unsigned char g_out_packet_buf[BUFFER_LENGTH];
unsigned char g_in_packet_buf[BUFFER_LENGTH];
unsigned int econdev_count;
unsigned int dev_node[10];
char	dev_name[64];

void close_hid(int hid_fd)
{
  if(hid_fd)
    close(hid_fd);
}

/* brief description on initExtensionUnit -Enumerating devices and getting hidnode 
 * param parameter - string parameter which contains hid type
 * return success/failure
 */

int initExtensionUnit(char * parameter)
{
	struct udev *udev;
	struct udev_enumerate *enumerate;
	struct udev_list_entry *devices, *dev_list_entry;
	struct udev_device *dev,*pdev;
	int	hid_fd;

	/* Create the udev object */
	udev = udev_new();
	if (!udev) {
		return FAILURE;
	}

	/* Create a list of the devices in the 'video4linux/hidraw' subsystem. */
	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, parameter);
	udev_enumerate_scan_devices(enumerate);
	devices = udev_enumerate_get_list_entry(enumerate);

	udev_list_entry_foreach(dev_list_entry, devices) {
		const char *path;       
		path = udev_list_entry_get_name(dev_list_entry);
		dev = udev_device_new_from_syspath(udev, path);
		pdev = udev_device_get_parent_with_subsystem_devtype(
								     dev,
								     "usb",
								     "usb_device");
		if (!pdev) {
			continue;
		}
		char *vidValue = ECON_VID, *pidValue = CAMERA_PID;
		if (strcmp(udev_device_get_sysattr_value(pdev,"idVendor"),vidValue) == 0)
		{     
			const char *hid_dev = udev_device_get_devnode(dev);
			hid_fd = open(hid_dev, O_RDWR|O_NONBLOCK);  

			if(hid_fd < 0)
				printf("Unable to open device \n");
			
			dev_node[econdev_count] = hid_fd;
			econdev_count++;
		}
		udev_device_unref(dev);

	}
	/* Free the enumerator object */
	udev_enumerate_unref(enumerate);
	udev_unref(udev);
	return SUCCESS;
}

/**
 * brief sendHidCmd - Sending hid command and get reply back
 * param outBuf - Buffer that fills to send into camera
 * param inBuf  - Buffer to get reply back
 * param len    - Buffer length
 * return success/failure
 * */
int sendHidCmd(unsigned char *outBuf, unsigned char *inBuf, int len, int hid_fd)
{
	// Write data into camera
	int ret = write(hid_fd, outBuf, len);
	if (ret < 0) {        
		perror("write");
		return FAILURE;
	}
	struct timeval tv;
	fd_set rfds;

	FD_ZERO(&rfds);
	FD_SET(hid_fd, &rfds);

	/* Wait up to 5 seconds. */
	tv.tv_sec = 5;
	tv.tv_usec = 0;

	// Monitor read file descriptor for 5 secs

	if(0 > select(1, &rfds, NULL, NULL, &tv)){
		perror("select");
		return FAILURE;
	}

	// Read data from camera
	int retval = read(hid_fd, inBuf, len);
	if (retval < 0) {
		perror("read");
		return FAILURE;
	}
	else{
		return SUCCESS;
	}
}


/**
 * brief initializeBuffers - Initialize input and output buffers
 */

void initializeBuffers(){
 //Initialize input and output buffers
 memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
 memset(g_in_packet_buf, 0x00, sizeof(g_in_packet_buf));
}

int setstream_mode(u_int8_t stream_Mode,int hid_fd)
{
    // hid validation
    if(hid_fd < 0)
    {
        return FAILURE;
    }

    //Initialize buffers
    initializeBuffers();

    // fill buffer values
    g_out_packet_buf[1] = CAMERA_CONTROL_SEE3CAM_20CUG  ; /* set camera control code */
    g_out_packet_buf[2] = SET_STREAM_MODE_20CUG; /* set stream mode command code */
    g_out_packet_buf[3] = stream_Mode; /* stream Mode */
    
    // send request and get reply from camera
    if(!sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH, hid_fd)){
        if (g_in_packet_buf[6]==SET_FAIL) {
            return FAILURE;
        } else if(g_in_packet_buf[0] == CAMERA_CONTROL_SEE3CAM_20CUG   &&
            g_in_packet_buf[1]==SET_STREAM_MODE_20CUG &&
            g_in_packet_buf[6]==SET_SUCCESS) {
            return SUCCESS;
        }
    }
    return FAILURE;
}

int getstream_mode(int hid_fd)
{
	// hid validation
	if(hid_fd < 0)
	{
		return FAILURE;
	}

	//Initialize buffers
	initializeBuffers();

	// fill buffer values
	g_out_packet_buf[1] = CAMERA_CONTROL_SEE3CAM_20CUG  ; /* set camera control code */
	g_out_packet_buf[2] = GET_STREAM_MODE_20CUG; /* get stream mode value */

	// send request and get reply from camera
	if(!sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH, hid_fd)){
		if (g_in_packet_buf[6]==GET_FAIL) {
			return FAILURE;
		} else if(g_in_packet_buf[0] == CAMERA_CONTROL_SEE3CAM_20CUG   &&
			  g_in_packet_buf[1]==GET_STREAM_MODE_20CUG &&
			  g_in_packet_buf[6]==GET_SUCCESS) {            
			return g_in_packet_buf[2];
		}
	}
	return FAILURE;
}

int main(int argc,char *argv[])
{
	int i = 0, choice = 0, option = 0, stream_mode = 0, stream_val = 0;

	if(initExtensionUnit("hidraw") < 0 || econdev_count == 0){
		printf("Device not found \n");
		return -1;
		}

	
	for(; i < econdev_count; i++) {
		sprintf(dev_name, "/dev/video%d", i);
		
		printf("%s \n", dev_name);
		printf("HIDRAW device node = %d \n", dev_node[i]);
		
		
		while (choice != 1) {
			printf("Enter the vaule to camera stream mode (1 = Set Camera Stream Mode, 2 = Get Camera Stream Mode):");
			scanf("%d", &option);
			switch (option) {
				case 1:
					printf("Enter the vaule to set camera stream mode (0 = MASTER_MODE, 1 = TRIGGER_MODE):");
					scanf("%d", &stream_mode);
					if (stream_mode < 0 || stream_mode > 1) 
					{
						printf("Wrong option. \t Enter correct option. \n");
					} 
					else 
					{
						int ret=setstream_mode(stream_mode,dev_node[i]);
						choice = 1;
                        if(ret == 0)
                        {
                            printf("Camera Stream Mode set successfully \n");
                        }
                        else
                        {
                            printf("Failed to set Camera Stream Mode \n");
                        }
                        
					}
				
				case 2:
					printf("Camera Stream Mode = %d \n", getstream_mode(dev_node[i]));
					choice = 1;
					break;
					
				default:
					printf("Wrong option. \t Enter correct option. \n");
					break;
			}
			
		}

		close_hid(dev_node[i]);
		
	}
	return 0;

}
