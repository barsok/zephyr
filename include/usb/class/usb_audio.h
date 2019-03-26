/* usb_audio.h - USB audio class public header */

/*
 * TODO
 */


/**
 * @file
 * @brief USB Audio Device Class public header
 *
 * todo; spec link
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_AUDIO_H_
#define ZEPHYR_INCLUDE_USB_CLASS_AUDIO_H_

#include <kernel.h>
//#include <usb/usb_common.h>



/**
 * @brief Helper macro for translating unsigned 24 bit value to 2 byte array.
 * */
#define U16_TO_BYTEARRAY(val) (uint8_t)(val),                    \
                                     (uint8_t)(((val) / (256)))

/**
 * @brief Helper macro for translating unsigned 24 bit value to 3 byte array.
 * */
#define U24_TO_BYTEARRAY(val) (uint8_t)(val),                    \
                                     (uint8_t)(((val) / (256))),        \
                                     (uint8_t)(((val) / (256 * 256)))

/**
 * @brief Helper macro for translating unsigned 32 bit value to 4 byte array.
 * */
#define U32_TO_BYTEARRAY(val) (uint8_t)(val),                    \
                                     (uint8_t)(((val) / (256))),        \
                                     (uint8_t)(((val) / (256 * 256))),  \
                                     (uint8_t)(((val) / (256 * 256 * 256)))











/**
 * @brief Possible values of feature unit control field.
 */
#define AUDIO_FEATUREUNIT_CONTROL_MUTE       (1u << 0) /**< Feature unit control bit MUTE      */
#define AUDIO_FEATUREUNIT_CONTROL_VOLUME     (1u << 1) /**< Feature unit control bit VOLUME    */
#define AUDIO_FEATUREUNIT_CONTROL_BASS       (1u << 2) /**< Feature unit control bit BASS      */
#define AUDIO_FEATUREUNIT_CONTROL_MID        (1u << 3) /**< Feature unit control bit MID       */
#define AUDIO_FEATUREUNIT_CONTROL_TREBLE     (1u << 4) /**< Feature unit control bit TREBLE    */
#define AUDIO_FEATUREUNIT_CONTROL_GRAPH_EQ   (1u << 5) /**< Feature unit control bit GRAPH_EQ  */
#define AUDIO_FEATUREUNIT_CONTROL_AUTO_GAIN  (1u << 6) /**< Feature unit control bit AUTO_GAIN */
#define AUDIO_FEATUREUNIT_CONTROL_DELAY      (1u << 7) /**< Feature unit control bit DELAY     */
#define AUDIO_FEATUREUNIT_CONTROL_BASS_BOOST (1u << 8) /**< Feature unit control bit BASS_BOOST*/
#define AUDIO_FEATUREUNIT_CONTROL_LOUDNESS   (1u << 9) /**< Feature unit control bit LOUDNESS  */



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





/*
struct audio_subclass_desc
{
   size_t                size;
   uint8_t               type;
   uint8_t const * const p_data;
};*/




#endif /* ZEPHYR_INCLUDE_USB_CLASS_AUDIO_H_ */
