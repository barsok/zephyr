#ifndef ZEPHYR_INCLUDE_USB_CLASS_AUDIO_DESC_H_
#define ZEPHYR_INCLUDE_USB_CLASS_AUDIO_DESC_H_

#include <usb/class/usb_audio.h>


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
#define DEFINE_AUDIO_FEATURE_UNIT_DESC_TYPE(name) \
	struct audio_feature_unit_desc_##name {	\
	    u8_t bLength;                	\
	    u8_t bDescriptorType;        	\
	    u8_t bDescriptorSubType;     	\
	    u8_t bUnitID;                	\
	    u8_t bSourceID;              	\
	    u8_t bControlSize;           	\
	    u8_t bmaControls[3];          	\
	    u8_t iFeature;			\
	} __packed;				\

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

#define DEFINE_AUDIO_HEADER_DESC_TYPE(name, param_if_count) \
	struct audio_header_desc_##name {	\
	    u8_t  bLength;			\
	    u8_t  bDescriptorType;		\
	    u8_t  bDescriptorSubType;		\
	    u16_t bcdADC;			\
	    u16_t wTotalLength;			\
	    u8_t  bInCollection;		\
	    u8_t  baInterfaceNr[param_if_count];\
	} __packed;



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


/*
 * 0 bLength 1 0x0C Size of this descriptor in bytes
1 bDescriptorType 1 0x24 Class specific INTERFACE descriptor
2 bDescriptorSubtype 1 0x02 INPUT_TERMINAL descriptor subtype
3 bTerminalID 1 0x04 Unique ID of this terminal set to 4
4 wTerminalType 2 0x0201 Terminal type is Microphone device
6 bAssocTerminal 1 0x00 No Output Terminal to which this Input Terminal is
associated
7 bNrChannels 1 0x01 Number of logical output channels in the Terminal’s output
audio channel cluster
8 wChannelConfig 2 0x0004 Bitmap location of the logical channels
10 iChannelNames 1 0x00 No non-predefined logical channels, the index is set to 0
11 iTerminal 1 0x00 No string descriptors for this Input Terminal
 */




/* Table 5-8. Microphone - Input Terminal Descriptor */
#define MIC_INITIALIZER_IFC_INPUT_TERMINAL(terminal_id) \
		{  \
			.bLength = sizeof(struct audio_input_terminal_desc), \
			.bDescriptorType = USB_CS_INTERFACE_DESC, \
			.bDescriptorSubType = AUDIO_CS_INPUT_TERMINAL, \
			.bTerminalID = terminal_id, \
			.wTerminalType = sys_cpu_to_le16(0x0201), \
			.bAssocTerminal = 0, \
			.bNrChannels = 2, \
			.wChannelConfig = sys_cpu_to_le16(0x0004), /* baso: nie wiem */ \
			.iChannelNames = 0, \
			.iTerminal = 0, \
		}



/* Table 5-7. Headphone - Input Terminal Descriptor */
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
		.wChannelConfig = sys_cpu_to_le16(0x0003),	/* left, right */\
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

/* Table 5-9. Mixer - Feature Unit Descriptor  */
#define INITIALIZER_IFC_FEATURE_UNIT(name, unit_id, source_id)		\
	{								\
		.bLength = sizeof(struct audio_feature_unit_desc_##name),\
		.bDescriptorType = USB_CS_INTERFACE_DESC,		\
		.bDescriptorSubType = AUDIO_CS_FEATURE_UNIT,		\
		.bUnitID = unit_id,					\
		.bSourceID = source_id,					\
		.bControlSize = 1,					\
		.bmaControls = { 0x01, 0x00, 0x01 },			\
		.iFeature = 0						\
	}


/* Table 5-5. Class Specific AudioControl Interface Descriptor */
//TODO: parameter for size
#define INITIALIZER_IFC_HEADER(name)				\
	{								\
		.bLength = sizeof(struct audio_header_desc_##name),	\
		.bDescriptorType = USB_CS_INTERFACE_DESC,		\
		.bDescriptorSubType = AUDIO_CS_HEADER,			\
		.bcdADC = sys_cpu_to_le16(0x0100),			\
		.wTotalLength = sys_cpu_to_le16(sizeof(struct usb_audio_config_control_##name)),		\
		.bInCollection = 1,					/* baso: tu ilosc interfejsow IN */ \
		.baInterfaceNr = {1}	\
	}

#define INITIALIZER_EP_DATA(cb, addr)					\
	{								\
		.ep_cb = cb,						\
		.ep_addr = addr,					\
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

/* Standard Audio Control Interface Descriptor */
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


#endif /* ZEPHYR_INCLUDE_USB_CLASS_AUDIO_DESC_H_ */

