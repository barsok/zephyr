#include <kernel.h>
#include <usb/usb_common.h>
#include <usb/usb_device.h>
#include <usb/class/usb_audio.h>

#include <misc/byteorder.h>

#define CONFIG_AUDIO_EP_SIZE	192





#define AUDIO_IN_TERM_CHCONF_LEFT_FRONT         (1u << 0)  /**< Channel config bit LEFT_FRONT      */
#define AUDIO_IN_TERM_CHCONF_RIGHT_FRONT        (1u << 1)  /**< Channel config bit RIGHT_FRONT     */
#define AUDIO_IN_TERM_CHCONF_CENTER_FRONT       (1u << 2)  /**< Channel config bit CENTER_FRONT    */
#define AUDIO_IN_TERM_CHCONF_LOW_FREQ_ENH       (1u << 3)  /**< Channel config bit LOW_FREQ_ENH    */
#define AUDIO_IN_TERM_CHCONF_LEFT_SURROUND      (1u << 4)  /**< Channel config bit LEFT_SURROUND   */
#define AUDIO_IN_TERM_CHCONF_RIGHT_SURROUND     (1u << 5)  /**< Channel config bit RIGHT_SURROUND  */
#define AUDIO_IN_TERM_CHCONF_LEFT_OF_CENTER     (1u << 6)  /**< Channel config bit LEFT_OF_CENTER  */
#define AUDIO_IN_TERM_CHCONF_RIGHT_OF_CENTER    (1u << 7)  /**< Channel config bit RIGHT_OF_CENTER */
#define AUDIO_IN_TERM_CHCONF_SURROUND           (1u << 8)  /**< Channel config bit SURROUND        */
#define AUDIO_IN_TERM_CHCONF_SIDE_LEFT          (1u << 9)  /**< Channel config bit SIDE_LEFT       */
#define AUDIO_IN_TERM_CHCONF_SIDE_RIGHT         (1u << 10) /**< Channel config bit SIDE_RIGHT      */
#define AUDIO_IN_TERM_CHCONF_TOP                (1u << 11) /**< Channel config bit TOP             */




struct usb_audio_ep_descriptor {
	u8_t bLength;
	u8_t bDescriptorType;
	u8_t bEndpointAddress;
	u8_t bmAttributes;
	u16_t wMaxPacketSize;
	u8_t bInterval;
	u8_t bRefresh;
	u8_t bSynchAddress;
} __packed;

/**
 * @brief Audio class input terminal descriptor.
 */
struct audio_input_terminal_desc {
    u8_t bLength;             //!< Length of the descriptor
    u8_t bDescriptorType;     //!< Descriptor type @ref APP_USBD_AUDIO_DESCRIPTOR_INTERFACE
    u8_t bDescriptorSubType;  //!< Descriptor subtype @ref APP_USBD_AUDIO_AC_IFACE_SUBTYPE_INPUT_TERMINAL
    u8_t bTerminalID;         //!< Terminal ID
    u16_t wTerminalType;      //!< Terminal type
    u8_t bAssocTerminal;      //!< Association terminal
    u8_t bNrChannels;         //!< Number of channels
    u16_t wChannelConfig;     //!< Channel config
    u8_t iChannelNames;       //!< Channel names
    u8_t iTerminal;           //!< Terminal string ID
};
/**
 * @brief Audio class output terminal descriptor.
 */
struct audio_output_terminal_desc {
    u8_t bLength;            //!< Length of the descriptor
    u8_t bDescriptorType;    //!< Descriptor type @ref APP_USBD_AUDIO_DESCRIPTOR_INTERFACE
    u8_t bDescriptorSubType; //!< Descriptor subtype @ref APP_USBD_AUDIO_AC_IFACE_SUBTYPE_OUTPUT_TERNINAL
    u8_t bTerminalID;        //!< Terminal ID
    u16_t wTerminalType;     //!< Terminal type
    u8_t bAssocTerminal;     //!< Association terminal
    u8_t bSourceID;          //!< Source ID
    u8_t iTerminal;          //!< Terminal string ID
} __packed;;

/**
 * @brief Audio class feature unit descriptor.
 */
struct audio_feature_unit_desc {
    u8_t bLength;                //!< Length of the descriptor
    u8_t bDescriptorType;        //!< Descriptor type @ref APP_USBD_AUDIO_DESCRIPTOR_INTERFACE
    u8_t bDescriptorSubType;     //!< Descriptor subtype @ref APP_USBD_AUDIO_AC_IFACE_SUBTYPE_FEATURE_UNIT
    u8_t bUnitID;                //!< Unit ID
    u8_t bSourceID;              //!< Source ID
    u8_t bControlSize;           //!< Control size
    u8_t bmaControls[];          //!< Controls array
} __packed;


/**
 * @brief Audio class control interface header descriptor.
 */
struct audio_header_desc {
    u8_t  bLength;            //!< Length of the descriptor
    u8_t  bDescriptorType;    //!< Descriptor type
    u8_t  bDescriptorSubType; //!< Descriptor subtype
    u16_t bcdADC;             //!< BCD ADC
    u16_t wTotalLength;       //!< Total interfaces length
    u8_t  bInCollection;      //!< Input collection
    u8_t  baInterfaceNr[];      //!< Interface number list
} ;






/**
 * @brief Audio class audio streaming interface descriptor.
 */
struct audio_as_iface_desc {
    u8_t  bLength;               //!< Length of the descriptor
    u8_t  bDescriptorType;       //!< Descriptor type @ref APP_USBD_AUDIO_DESCRIPTOR_INTERFACE
    u8_t  bDescriptorSubType;    //!< Descriptor subtype @ref app_usbd_audio_ac_iface_subtype_t
    u8_t  bTerminalLink;         //!< Terminal link
    u8_t  bDelay;                //!< Delay
    u16_t wFormatTag;            //!< Format TAG
} __packed;

/**
 * @brief Audio class audio endpoint descriptor.
 */
struct audio_as_endpoint_desc {
    u8_t  bLength;               //!< Length of the descriptor
    u8_t  bDescriptorType;       //!< Descriptor type @ref APP_USBD_AUDIO_DESCRIPTOR_ENDPOINT
    u8_t  bDescriptorSubType;    //!< Descriptor subtype @ref APP_USBD_AUDIO_EP_SUBTYPE_GENERAL
    u8_t  bmAttributes;          //!< Audio endpoint attributes
    u8_t  bLockDelayUnits;       //!< Lock delay units
    u16_t wLockDelay;            //!< Lock delay value
} __packed;

/**
 * @brief Audio class audio streaming format type I descriptor.
 */
struct audio_as_format_type_i_desc {
    u8_t bLength;                //!< Length of the descriptor
    u8_t bDescriptorType;        //!< Descriptor type @ref APP_USBD_AUDIO_DESCRIPTOR_INTERFACE
    u8_t bDescriptorSubType;     //!< Descriptor subtype @ref app_usbd_audio_as_iface_subtype_t
    u8_t bFormatType;            //!< Format type: fixed value 1
    u8_t bNrChannels;            //!< Number of channels
    u8_t bSubframeSize;          //!< Subframe size
    u8_t bBitResolution;         //!< Bit resolution
    u8_t bSamFreqType;           //!< Number of supported sampling frequencies
    u8_t tSamFreq[];             //!< Number of supported sampling frequencies table (24 bit entries)
} __packed;


/**
 * @brief Audio class audio streaming format type II descriptor.
 */
struct audio_as_format_type_ii_desc {
    u8_t bLength;                //!< Length of the descriptor
    u8_t bDescriptorType;        //!< Descriptor type @ref APP_USBD_AUDIO_DESCRIPTOR_INTERFACE
    u8_t bDescriptorSubType;     //!< Descriptor subtype @ref app_usbd_audio_as_iface_subtype_t
    u8_t bFormatType;            //!< Format type: fixed value 2
    u16_t wMaxBitRate;           //!< Maximum bitrate
    u16_t wSamplesPerFrame;      //!< Samples per frame
    u8_t bSamFreqType;           //!< Number of supported sampling frequencies
    u8_t tSamFreq[];             //!< Number of supported sampling frequencies table (24 bit entries)
} __packed;

/**
 * @brief Audio class audio streaming format type III descriptor.
 */
struct audio_as_format_type_iii_desc {
    uint8_t bLength;              //!< Length of the descriptor
    uint8_t bDescriptorType;      //!< Descriptor type @ref APP_USBD_AUDIO_DESCRIPTOR_INTERFACE
    uint8_t bDescriptorSubType;   //!< Descriptor subtype @ref app_usbd_audio_as_iface_subtype_t
    uint8_t bFormatType;          //!< Format type: fixed value 1
    uint8_t bNrChannels;          //!< Number of channels
    uint8_t bSubframeSize;        //!< Subframe size
    uint8_t bBitResolution;       //!< Bit resolution
    uint8_t bSamFreqType;         //!< Number of supported sampling frequencies
    uint8_t tSamFreq[];           //!< Number of supported sampling frequencies table (24 bit entries)
} __packed;


#define DEFINE_AUDIO_FORMAT_I_TYPE(name, freq_cnt)	\
	struct audio_as_format_type_i_desc_##name {	\
	    u8_t bLength;                \
	    u8_t bDescriptorType;        \
	    u8_t bDescriptorSubType;     \
	    u8_t bFormatType;            \
	    u8_t bNrChannels;            \
	    u8_t bSubframeSize;          \
	    u8_t bBitResolution;         \
	    u8_t bSamFreqType;           \
	    u8_t tSamFreq[freq_cnt * 3]; \
	} __packed;

#define DEFINE_AUDIO_FORMAT_II_TYPE(name, freq_cnt)	\
	struct audio_as_format_type_ii_desc_##name {	\
	    u8_t bLength;                \
	    u8_t bDescriptorType;        \
	    u8_t bDescriptorSubType;     \
	    u8_t bFormatType;            \
	    u16_t wMaxBitRate;           \
	    u16_t wSamplesPerFrame;      \
	    u8_t bSamFreqType;           \
	    u8_t tSamFreq[freq_cnt * 3]; \
	} __packed;

#define DEFINE_AUDIO_AS_FORMAT_III_TYPE(name, freq_cnt)	\
struct audio_as_format_type_iii_desc_##name {		\
    uint8_t bLength;                \
    uint8_t bDescriptorType;        \
    uint8_t bDescriptorSubType;     \
    uint8_t bFormatType;            \
    uint8_t bNrChannels;            \
    uint8_t bSubframeSize;          \
    uint8_t bBitResolution;         \
    uint8_t bSamFreqType;           \
    uint8_t tSamFreq[freq_cnt * 3]; \
} __packed;




#define INITIALIZER_IFS_FORMAT_III(name)			\
	{				 			\
		.bLength = sizeof(struct audio_as_format_type_iii_desc_##name),\
		.bDescriptorType = USB_CS_INTERFACE_DESC,	\
		.bDescriptorSubType = AUDIO_FORMAT_TYPE,	\
		.bFormatType = 0x03,				\
		.bNrChannels = 2,				\
		.bSubframeSize = 2,				\
		.bBitResolution = 16,				\
		.bSamFreqType = 1,				\
		.tSamFreq = {0x80, 0xBB, 0x00}			\
	}


#define INITIALIZER_IFS_AS_IFACE(terminal_id)				\
{									\
    .bLength		= sizeof(struct audio_as_iface_desc),	\
    .bDescriptorType	= USB_CS_INTERFACE_DESC,			\
    .bDescriptorSubType	= AUDIO_AS_GENERAL,				\
    .bTerminalLink	= (terminal_id),				\
    .bDelay		= 0,					\
    .wFormatTag		= sys_cpu_to_le16(1)			\
}

#define INITIALIZER_IFS_AS_ENDPOINT					\
{									\
    .bLength		= sizeof(struct audio_as_endpoint_desc),	\
    .bDescriptorType	= USB_CS_ENDPOINT_DESC,			\
    .bDescriptorSubType	= 0x01,					\
    .bmAttributes	= 0,					\
    .bLockDelayUnits	= 0,					\
    .wLockDelay		= sys_cpu_to_le16(0)			\
}





//todo: terminaltype, n_channels, channelconfig
#define INITIALIZER_IFC_INPUT_TERMINAL(terminal_id)		\
	{								\
		.bLength = sizeof(struct audio_input_terminal_desc),	\
		.bDescriptorType = USB_CS_INTERFACE_DESC,		\
		.bDescriptorSubType = AUDIO_CS_INPUT_TERMINAL,		\
		.bTerminalID = terminal_id,				\
		.wTerminalType = sys_cpu_to_le16(0x0101),		\
		.bAssocTerminal = 0,					\
		.bNrChannels = 2, 				\
		.wChannelConfig = sys_cpu_to_le16(0x0003),	\
		.iChannelNames = 0,					\
		.iTerminal = 0						\
	}
//todo: terminal type
#define INITIALIZER_IFC_OUTPUT_TERMINAL(terminal_id, source_id)		\
	{								\
		.bLength = sizeof(struct audio_output_terminal_desc),	\
		.bDescriptorType = USB_CS_INTERFACE_DESC,		\
		.bDescriptorSubType = AUDIO_CS_OUTPUT_TERMINAL,		\
		.bTerminalID = terminal_id,				\
		.wTerminalType = sys_cpu_to_le16(0x0302),			\
		.bAssocTerminal = 0,					\
		.bSourceID = source_id,					\
		.iTerminal = 0						\
	}

#define INITIALIZER_IFC_FEATURE_UNIT(name, unit_id, source_id)		\
	{								\
		.bLength = sizeof(struct audio_feature_unit_desc_##name),\
		.bDescriptorType = USB_CS_INTERFACE_DESC,		\
		.bDescriptorSubType = AUDIO_CS_FEATURE_UNIT,		\
		.bUnitID = unit_id,					\
		.bSourceID = source_id,					\
		.bControlSize = 1,					\
		.bmaControls = { sys_cpu_to_le16(0x0001),		\
				 sys_cpu_to_le16(0x0001),		\
				 sys_cpu_to_le16(0x0001) },		\
		.iFeature = 0						\
	}


//TODO: parameter for size
#define INITIALIZER_IFC_HEADER(name)				\
	{								\
		.bLength = sizeof(struct audio_header_desc_##name),	\
		.bDescriptorType = USB_CS_INTERFACE_DESC,		\
		.bDescriptorSubType = AUDIO_CS_HEADER,			\
		.bcdADC = sys_cpu_to_le16(0x0100),			\
		.wTotalLength = sys_cpu_to_le16(sizeof(struct usb_audio_config_control_##name)),		\
		.bInCollection = 1,					\
		.baInterfaceNr = {1}	\
	}

#define INITIALIZER_EP_DATA(cb, addr)					\
	{								\
		.ep_cb = cb,						\
		.ep_addr = addr,					\
	}

#define DEFINE_AUDIO_EP(x, stream_ep_addr)	\
	static struct usb_ep_cfg_data usb_audio_ep_data_##x[] = {	\
		INITIALIZER_EP_DATA(usb_audio_ep_cb, stream_ep_addr),		\
	}

#define DEFINE_AUDIO_DATA(x)					\
	USBD_CFG_DATA_DEFINE(usb_audio)					\
	struct usb_cfg_data usb_audio_config_##x = {			\
		.usb_device_description = NULL,				\
		.interface_config = audio_interface_config,		\
		.interface_descriptor = &usb_audio_desc_##x.if_control,	\
		.cb_usb_status_composite =				\
				audio_status_composite_cb,		\
		.interface = {						\
			.class_handler = NULL,	\
			.custom_handler = NULL,				\
			.payload_data = NULL,				\
		},							\
		.num_endpoints = ARRAY_SIZE(usb_audio_ep_data_##x),	\
		.endpoint = usb_audio_ep_data_##x,			\
	}

#define INITIALIZER_IF_EP(addr, attr, mps, interval)			\
	{								\
		.bLength = sizeof(struct usb_audio_ep_descriptor),		\
		.bDescriptorType = USB_ENDPOINT_DESC,			\
		.bEndpointAddress = addr,				\
		.bmAttributes = attr,					\
		.wMaxPacketSize = sys_cpu_to_le16(mps),			\
		.bInterval = interval,					\
	}

#define INITIALIZER_IF_CONTROL						\
	{								\
		.bLength = sizeof(struct usb_if_descriptor),		\
		.bDescriptorType = USB_INTERFACE_DESC,			\
		.bInterfaceNumber = 0,					\
		.bAlternateSetting = 0,					\
		.bNumEndpoints = 0,					\
		.bInterfaceClass = AUDIO_CLASS,				\
		.bInterfaceSubClass = AUDIO_CONTROL_SUBCLASS,		\
		.bInterfaceProtocol = 0,				\
		.iInterface = 0,					\
	}
#define INITIALIZER_IF_STREAMING_NON_ISO				\
	{								\
		.bLength = sizeof(struct usb_if_descriptor),		\
		.bDescriptorType = USB_INTERFACE_DESC,			\
		.bInterfaceNumber = 1,					\
		.bAlternateSetting = 0,					\
		.bNumEndpoints = 0,					\
		.bInterfaceClass = AUDIO_CLASS,				\
		.bInterfaceSubClass = AUDIO_STREAMING_SUBCLASS,		\
		.bInterfaceProtocol = 0,				\
		.iInterface = 0,					\
	}
#define INITIALIZER_IF_STREAMING					\
	{								\
		.bLength = sizeof(struct usb_if_descriptor),		\
		.bDescriptorType = USB_INTERFACE_DESC,			\
		.bInterfaceNumber = 1,					\
		.bAlternateSetting = 1,					\
		.bNumEndpoints = 1,					\
		.bInterfaceClass = AUDIO_CLASS,				\
		.bInterfaceSubClass = AUDIO_STREAMING_SUBCLASS,		\
		.bInterfaceProtocol = 0,				\
		.iInterface = 0,					\
	}


#define DEFINE_AUDIO_HEADPHONES_DESCR(x, stream_ep_addr)\
	DEFINE_AUDIO_HEADPHONES_DESCR_TYPES(x, 1)			\
	USBD_CLASS_DESCR_DEFINE(primary, x)				\
	struct usb_audio_config_##x usb_audio_desc_##x = {		\
	.if_control = INITIALIZER_IF_CONTROL,				\
	.control_desc = {						\
		.header = INITIALIZER_IFC_HEADER(x),			\
		.input_terminal = INITIALIZER_IFC_INPUT_TERMINAL(1),	\
		.feature = INITIALIZER_IFC_FEATURE_UNIT(x, 2, 1) ,	\
		.output_terminal = INITIALIZER_IFC_OUTPUT_TERMINAL(3, 2),\
	},								\
	.if_streaming_non_iso = INITIALIZER_IF_STREAMING_NON_ISO,	\
	.if_streaming = INITIALIZER_IF_STREAMING,			\
	.streaming_desc = {						\
		.iface = INITIALIZER_IFS_AS_IFACE(1),			\
		.format = INITIALIZER_IFS_FORMAT_III(x),		\
		.ep = INITIALIZER_IFS_AS_ENDPOINT			\
	},								\
	.if_streaming_ep = INITIALIZER_IF_EP(stream_ep_addr,		\
					     USB_DC_EP_ISOCHRONOUS,	\
					     CONFIG_AUDIO_EP_SIZE,	\
					     0x01)			\
}

#define DEFINE_AUDIO_HEADPHONES_DESCR_TYPES(name, param_if_count) \
	DEFINE_AUDIO_AS_FORMAT_III_TYPE(name, 1)\
	struct audio_header_desc_##name {	\
	    u8_t  bLength;			\
	    u8_t  bDescriptorType;		\
	    u8_t  bDescriptorSubType;		\
	    u16_t bcdADC;			\
	    u16_t wTotalLength;			\
	    u8_t  bInCollection;		\
	    u8_t  baInterfaceNr[param_if_count];\
	} __packed;				\
	struct audio_feature_unit_desc_##name {	\
	    u8_t bLength;                	\
	    u8_t bDescriptorType;        	\
	    u8_t bDescriptorSubType;     	\
	    u8_t bUnitID;                	\
	    u8_t bSourceID;              	\
	    u8_t bControlSize;           	\
	    u16_t bmaControls[3];          	\
	    u8_t iFeature;			\
	} __packed;				\
	struct usb_audio_config_control_##name {			\
		struct audio_header_desc_##name  header;		\
		struct audio_input_terminal_desc input_terminal;	\
		struct audio_feature_unit_desc_##name feature;		\
		struct audio_output_terminal_desc output_terminal;	\
	} __packed;							\
	struct usb_audio_config_streaming_##name {			\
		struct audio_as_iface_desc iface;			\
		struct audio_as_format_type_iii_desc_##name  format;	\
		struct audio_as_endpoint_desc ep;			\
	} __packed;							\
	struct usb_audio_config_##name {				\
		struct usb_if_descriptor if_control;			\
		struct usb_audio_config_control_##name control_desc;	\
		struct usb_if_descriptor if_streaming_non_iso;		\
		struct usb_if_descriptor if_streaming;			\
		struct usb_audio_config_streaming_##name streaming_desc;\
		struct usb_audio_ep_descriptor if_streaming_ep;		\
	} __packed;








/*

struct audio_instance {

    struct audio_subclass_desc const * const p_format_dsc;  //!< Audio class Format descriptor
    struct audio_subclass_desc const * const p_input_dsc;   //!< Audio class Input Terminal descriptor
    struct audio_subclass_desc const * const p_output_dsc;  //!< Audio class Output Terminal descriptor
    struct audio_subclass_desc const * const p_feature_dsc; //!< Audio class Feature Unit descriptor

    uint8_t  delay;     //!< Streaming delay
    uint16_t format;    //!< FormatTag (@ref app_usbd_audio_as_iface_format_tag_t)
    uint16_t ep_size;   //!< Endpoint size

    //app_usbd_audio_subclass_t type_streaming;   //!< Streaming type MIDISTREAMING/AUDIOSTREAMING (@ref app_usbd_audio_subclass_t)

    //app_usbd_audio_user_ev_handler_t user_ev_handler; //!< User event handler
};

*/


static void audio_interface_config(struct usb_desc_header *head,
				      u8_t bInterfaceNumber)
{
	ARG_UNUSED(head);

	//loopback_cfg.if0.bInterfaceNumber = bInterfaceNumber;
}

static void audio_status_composite_cb(struct usb_cfg_data *cfg,
					    enum usb_dc_status_code status,
					    const u8_t *param)
{
}

static void usb_audio_ep_cb(u8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{

}







DEFINE_AUDIO_EP(0,0x00);
DEFINE_AUDIO_HEADPHONES_DESCR(0,0x00);
DEFINE_AUDIO_DATA(0);

