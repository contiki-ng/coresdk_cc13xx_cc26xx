/*
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       TRNG.h
 *
 *  @brief      TRNG driver header
 *
 *  @warning    This is a beta API. It may change in future releases.
 *
 *  # Overview #
 *  The True Random Number Generator (TRNG) module generates numbers of variable
 *  lengths from a source of entropy. The output is suitable for applications
 *  requiring cryptographically random numbers such as keying material for
 *  private or symmetric keys.
 *
 *
 *  # Usage #
 *
 *  ## Before starting a TRNG operation #
 *
 *  Before starting a TRNG operation, the application must do the following:
 *      - Call TRNG_init() to initialize the driver.
 *      - Call TRNG_Params_init() to initialize the TRNG_Params to default values.
 *      - Modify the TRNG_Params as desired.
 *      - Call TRNG_open() to open an instance of the driver.
 *      - Initialize a blank CryptoKey. These opaque datastructures are representations
 *        of keying material and its storage. Depending on how the keying material
 *        is stored (RAM or flash, key store, key blob), the CryptoKey must be
 *        initialized differently. The TRNG API can handle all types of CryptoKey.
 *        However, not all device-specific implementions support all types of CryptoKey.
 *        Devices without a key store will not support CryptoKeys with keying material
 *        stored in a key store for example.
 *        All devices support plaintext CryptoKeys.
 *
 *  ## TRNG operations #
 *
 *  TRNG_generateEntropy() provides the most basic functionality. Use it to
 *  generate random numbers of a specified width without further restrictions.
 *  An example use-case would be generating a symmetric key for AES encryption
 *  and / or authentication.
 *
 *  TRNG_generateEntropyLessThan() returns a random number with the restriction
 *  that the random number be less than a specified value. The algorithm used
 *  to ensure negligible biasing of the resultant random number is implementation
 *  dependent.
 *
 *  TRNG_generateEntropyNonZeroLessThan() also returns a random number with
 *  the restriction that the number be less than a specified value. Further,
 *  the number will not be zero either. This call specifically is useful
 *  if you are trying to generate a private key for use with elliptic curve
 *  cryptography. Private keys commonly have the restriction that they be
 *  within [1, n - 1], where n is the order of the curve. This function guarantees
 *  that you will have an unbiased number in that range when it returns.
 *  The algorithm used to ensure negligible biasing of the resultant
 *  random number is implementation dependent.
 *
 *  While TRNG_generateEntropyNonZeroLessThan() is guaranteed to produce
 *  entropy fit for use in ECC operations, it may not be the most sensible choice.
 *  TRNG_generateEntropyNonZeroLessThan() requires overhead both
 *  in code size and in run-time and thus power consumption. The order of
 *  a curve is often a large number very close to the upper bound of numbers
 *  that fit the curve parameter width. This means that, for many curves, it
 *  is improbable that a randomly generated number is an invalid private key.
 *  The ECDH public key generation functions will reject invalid private keys
 *  with an error code. This lets you implement rejection sampling by using
 *  the basic TRNG_generateEntropy() to generate a random number and simply
 *  generating a new one if the ECDH public key generation function rejects
 *  it.
 *
 *  Not all implementations support the more specialized functions as they require
 *  efficient operations on large numbers. Usually, this means the device needs
 *  a large number maths accelerator or public key accelerator.
 *
 *  ## After the TRNG operation completes #
 *
 *  After the TRNG operation completes, the application should either start another operation
 *  or close the driver by calling TRNG_close().
 *
 *  ## TRNG Driver Configuration #
 *
 *  In order to use the TRNG APIs, the application is required
 *  to provide device-specific TRNG configuration in the Board.c file.
 *  The TRNG driver interface defines a configuration data structure:
 *
 *  @code
 *  typedef struct TRNG_Config_ {
 *      void                   *object;
 *      void          const    *hwAttrs;
 *  } TRNG_Config;
 *  @endcode
 *
 *  The application must declare an array of TRNG_Config elements, named
 *  TRNG_config[].  Each element of TRNG_config[] must be populated with
 *  pointers to a device specific TRNG driver implementation's driver object and
 *  hardware attributes.  The hardware attributes define properties such
 *  as the TRNG peripheral's base address.
 *  Each element in TRNG_config[] corresponds to a TRNG instance
 *  and none of the elements should have NULL pointers.
 *  There is no correlation between the index and the
 *  peripheral designation (such as TRNG0 or TRNG1).  For example, it is
 *  possible to use TRNG_config[0] for TRNG1. Multiple drivers and driver
 *  instances may all access the same underlying hardware. This is transparent
 *  to the application. Mutual exclusion is performed automatically by the
 *  drivers as necessary.
 *
 *  Because the TRNG configuration is very device dependent, you will need to
 *  check the doxygen for the device specific TRNG implementation.  There you
 *  will find a description of the TRNG hardware attributes.  Please also
 *  refer to the Board.c file of any of your examples to see the TRNG
 *  configuration.
 *
 *  ## TRNG Parameters
 *
 *  The #TRNG_Params structure is passed to the TRNG_open() call.  If NULL
 *  is passed for the parameters, TRNG_open() uses default parameters.
 *  A #TRNG_Params structure is initialized with default values by passing
 *  it to TRNG_Params_init().
 *  Some of the TRNG parameters are described below.  To see brief descriptions
 *  of all the parameters, see #TRNG_Params.
 *
 *  ## Examples
 *
 *  ### Generate symmetric encryption key #
 *  @code
 *
 *  #include <ti/drivers/TRNG.h>
 *  #include <ti/drivers/cryptoutils/cryptokey/CryptoKeyPlaintext.h>
 *
 *  #define KEY_LENGTH_BYTES 16
 *
 *  TRNG_Handle handle;
 *  int_fast16_t result;
 *
 *  CryptoKey entropyKey;
 *  uint8_t entropyBuffer[KEY_LENGTH_BYTES];
 *
 *  handle = TRNG_open(0, NULL);
 *
 *  if (!handle) {
 *      // Handle error
 *      while(1);
 *  }
 *
 *  CryptoKeyPlaintext_initBlankKey(&entropyKey, entropyBuffer, KEY_LENGTH_BYTES);
 *
 *  result = TRNG_generateEntropy(handle, &entropyKey);
 *
 *  if (result != TRNG_STATUS_SUCCESS) {
 *      // Handle error
 *      while(1);
 *  }
 *
 *  TRNG_close(handle);
 *
 *  @endcode
 *
 *
 *  ### Generate ECC private key #
 *  @code
 *
 *  #include <ti/drivers/TRNG.h>
 *  #include <ti/drivers/cryptoutils/cryptokey/CryptoKeyPlaintext.h>
 *  #include <ti/drivers/cryptoutils/ecc/ECCParams.h>
 *
 *  // The NIST-P256 curve has 256-bit curve parameters and thus 32-byte private
 *  // keys.
 *  #define PRIVATE_KEY_LENGTH_BYTES 32
 *
 *  TRNG_Handle handle;
 *  int_fast16_t result;
 *
 *  CryptoKey entropyKey;
 *  uint8_t entropyBuffer[PRIVATE_KEY_LENGTH_BYTES];
 *
 *  handle = TRNG_open(0, NULL);
 *
 *  if (!handle) {
 *      // Handle error
 *      while(1);
 *  }
 *
 *  CryptoKeyPlaintext_initBlankKey(&entropyKey, entropyBuffer, ECCParams_NISTP256.length);
 *
 *  result = TRNG_generateEntropyNonZeroLessThan(handle,
 *                                               &entropyKey,
 *                                               ECCParams_NISTP256.order);
 *
 *  if (result != TRNG_STATUS_SUCCESS) {
 *      // Handle error
 *      while(1);
 *  }
 *
 *  TRNG_close(handle);
 *
 *  @endcode
 *
 *
 *  ### Generate ECC private and public key using rejection sampling #
 *  @code
 *
 *  #include <ti/drivers/TRNG.h>
 *  #include <ti/drivers/ECDH.h>
 *  #include <ti/drivers/cryptoutils/cryptokey/CryptoKeyPlaintext.h>
 *  #include <ti/drivers/cryptoutils/ecc/ECCParams.h>
 *
 *  TRNG_Handle trngHandle;
 *  ECDH_Handle ecdhHandle;
 *
 *  CryptoKey privateKey;
 *  CryptoKey publicKey;
 *
 *  int_fast16_t trngResult;
 *  int_fast16_t ecdhResult;
 *
 *  uint8_t privateKeyingMaterial[32];
 *  uint8_t publicKeyingMaterial[64];
 *
 *  ECDH_OperationGeneratePublicKey genPubKeyOperation;
 *
 *  trngHandle = TRNG_open(0, NULL);
 *  if (!trngHandle) {
 *      while(1);
 *  }
 *
 *  ecdhHandle = ECDH_open(0, NULL);
 *  if (!ecdhHandle) {
 *      while(1);
 *  }
 *
 *  // Repeatedly generate random numbers until they are in the range [1, n - 1].
 *  // Since the NIST-P256 order is so close to 2^256, the probability of needing
 *  // to generate more than one random number is incredibly low but not non-zero.
 *  do {
 *
 *      CryptoKeyPlaintext_initBlankKey(&privateKey, privateKeyingMaterial, ECCParams_NISTP256.length);
 *      CryptoKeyPlaintext_initBlankKey(&publicKey, publicKeyingMaterial, 2 * ECCParams_NISTP256.length);
 *
 *      trngResult = TRNG_generateEntropy(trngHandle, &privateKey);
 *
 *      if (trngResult != TRNG_STATUS_SUCCESS) {
 *          while(1);
 *      }
 *
 *      ECDH_OperationGeneratePublicKey_init(&genPubKeyOperation);
 *      genPubKeyOperation.curve = &ECCParams_NISTP256;
 *      genPubKeyOperation.myPrivateKey = &privateKey;
 *      genPubKeyOperation.myPublicKey = &publicKey;
 *
 *      ecdhResult = ECDH_generatePublicKey(ecdhHandle, &genPubKeyOperation);
 *
 *  } while(ecdhResult == ECDH_STATUS_PRIVATE_KEY_LARGER_EQUAL_ORDER || ecdhResult == ECDH_STATUS_PRIVATE_KEY_ZERO);
 *
 *  TRNG_close(trngHandle);
 *  ECDH_close(ecdhHandle);
 *
 *  @endcode
 */

#ifndef ti_drivers_TRNG__include
#define ti_drivers_TRNG__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/drivers/cryptoutils/cryptokey/CryptoKey.h>

/**
 *  @defgroup TRNG_CONTROL TRNG_control command and status codes
 *  These TRNG macros are reservations for TRNG.h
 *  @{
 */

/*!
 * Common TRNG_control command code reservation offset.
 * TRNG driver implementations should offset command codes with TRNG_CMD_RESERVED
 * growing positively
 *
 * Example implementation specific command codes:
 * @code
 * #define TRNGXYZ_CMD_COMMAND0     TRNG_CMD_RESERVED + 0
 * #define TRNGXYZ_CMD_COMMAND1     TRNG_CMD_RESERVED + 1
 * @endcode
 */
#define TRNG_CMD_RESERVED           (32)

/*!
 * Common TRNG_control status code reservation offset.
 * TRNG driver implementations should offset status codes with
 * TRNG_STATUS_RESERVED growing negatively.
 *
 * Example implementation specific status codes:
 * @code
 * #define TRNGXYZ_STATUS_ERROR0    TRNG_STATUS_RESERVED - 0
 * #define TRNGXYZ_STATUS_ERROR1    TRNG_STATUS_RESERVED - 1
 * #define TRNGXYZ_STATUS_ERROR2    TRNG_STATUS_RESERVED - 2
 * @endcode
 */
#define TRNG_STATUS_RESERVED        (-32)

/**
 *  @defgroup TRNG_STATUS Status Codes
 *  TRNG_STATUS_* macros are general status codes returned by TRNG functions
 *  @{
 *  @ingroup TRNG_CONTROL
 */

/*!
 * @brief   Successful status code.
 *
 * Functions return TRNG_STATUS_SUCCESS if the function was executed
 * successfully.
 */
#define TRNG_STATUS_SUCCESS         (0)

/*!
 * @brief   Generic error status code.
 *
 * Functions return TRNG_STATUS_ERROR if the function was not executed
 * successfully.
 */
#define TRNG_STATUS_ERROR           (-1)

/*!
 * @brief   An error status code returned by TRNG_control() for undefined
 * command codes.
 *
 * TRNG_control() returns TRNG_STATUS_UNDEFINEDCMD if the control code is not
 * recognized by the driver implementation.
 */
#define TRNG_STATUS_UNDEFINEDCMD    (-2)

/*!
 * @brief   An error status code returned if the hardware or software resource
 * is currently unavailable.
 *
 * TRNG driver implementations may have hardware or software limitations on how
 * many clients can simultaneously perform operations. This status code is returned
 * if the mutual exclusion mechanism signals that an operation cannot currently be performed.
 */
#define TRNG_STATUS_RESOURCE_UNAVAILABLE (-3)

/** @}*/

/** @}*/

/**
 *  @defgroup TRNG_CMD Command Codes
 *  TRNG_CMD_* macros are general command codes for TRNG_control(). Not all TRNG
 *  driver implementations support these command codes.
 *  @{
 *  @ingroup TRNG_CONTROL
 */

/* Add TRNG_CMD_<commands> here */

/** @}*/

/** @}*/

/*!
 *  @brief  A handle that is returned from a TRNG_open() call.
 */
typedef struct TRNG_Config_    *TRNG_Handle;

/*!
 * @brief   The way in which TRNG function calls return after generating
 *          the requested entropy.
 *
 * Not all TRNG operations exhibit the specified return behavor. Functions that do not
 * require significant computation and cannot offload that computation to a background thread
 * behave like regular functions. Which functions exhibit the specfied return behavior is not
 * implementation dependent. Specifically, a software-backed implementation run on the same
 * CPU as the application will emulate the return behavior while not actually offloading
 * the computation to the background thread.
 *
 * TRNG functions exhibiting the specified return behavior have restrictions on the
 * context from which they may be called.
 *
 * |                              | Task  | Hwi   | Swi   |
 * |------------------------------|-------|-------|-------|
 * |TRNG_RETURN_BEHAVIOR_CALLBACK | X     | X     | X     |
 * |TRNG_RETURN_BEHAVIOR_BLOCKING | X     |       |       |
 * |TRNG_RETURN_BEHAVIOR_POLLING  | X     | X     | X     |
 *
 */
typedef enum TRNG_ReturnBehavior_ {
    TRNG_RETURN_BEHAVIOR_CALLBACK = 1,    /*!< The function call will return immediately while the
                                             *   TRNG operation goes on in the background. The registered
                                             *   callback function is called after the operation completes.
                                             *   The context the callback function is called (task, HWI, SWI)
                                             *   is implementation-dependent.
                                             */
    TRNG_RETURN_BEHAVIOR_BLOCKING = 2,    /*!< The function call will block while TRNG operation goes
                                             *   on in the background. TRNG operation results are available
                                             *   after the function returns.
                                             */
    TRNG_RETURN_BEHAVIOR_POLLING  = 4,    /*!< The function call will continuously poll a flag while TRNG
                                             *   operation goes on in the background. TRNG operation results
                                             *   are available after the function returns.
                                             */
} TRNG_ReturnBehavior;

/*!
 *  @brief TRNG Global configuration
 *
 *  The TRNG_Config structure contains a set of pointers used to characterize
 *  the TRNG driver implementation.
 *
 *  This structure needs to be defined before calling TRNG_init() and it must
 *  not be changed thereafter.
 *
 *  @sa     TRNG_init()
 */
typedef struct TRNG_Config_ {
    /*! Pointer to a driver specific data object */
    void               *object;

    /*! Pointer to a driver specific hardware attributes structure */
    void         const *hwAttrs;
} TRNG_Config;

/*!
 *  @brief  The definition of a callback function used by the TRNG driver
 *          when used in ::TRNG_RETURN_BEHAVIOR_CALLBACK
 *
 *  @param  handle  Handle of the client that started the TRNG operation.
 *
 *  @param  returnValue Return status code describing the outcome of the operation.
 *
 *  @param  entropy     The CryptoKey that describes the location the generated
 *                      entropy will be copied to.
 */
typedef void (*TRNG_CallbackFxn) (TRNG_Handle handle,
                                  int_fast16_t returnValue,
                                  CryptoKey *entropy);

/*!
 *  @brief  TRNG Parameters
 *
 *  TRNG Parameters are used to with the TRNG_open() call. Default values for
 *  these parameters are set using TRNG_Params_init().
 *
 *  @sa     TRNG_Params_init()
 */
typedef struct TRNG_Params_ {
    TRNG_ReturnBehavior     returnBehavior;             /*!< Blocking, callback, or polling return behavior */
    TRNG_CallbackFxn        callbackFxn;                /*!< Callback function pointer */
    uint32_t                timeout;                    /*!< Timeout before the driver returns an error in
                                                         *   ::TRNG_RETURN_BEHAVIOR_BLOCKING
                                                         */
    void                   *custom;                     /*!< Custom argument used by driver
                                                         *   implementation
                                                         */
} TRNG_Params;

/*!
 *  @brief Default TRNG_Params structure
 *
 *  @sa     TRNG_Params_init()
 */
extern const TRNG_Params TRNG_defaultParams;

/*!
 *  @brief  This function initializes the TRNG module.
 *
 *  @pre    The TRNG_config structure must exist and be persistent before this
 *          function can be called. This function must also be called before
 *          any other TRNG driver APIs. This function call does not modify any
 *          peripheral registers.
 */
void TRNG_init(void);

/*!
 *  @brief  Function to initialize the TRNG_Params struct to its defaults
 *
 *  @param  params      An pointer to TRNG_Params structure for
 *                      initialization
 *
 *  Defaults values are:
 *      returnBehavior              = TRNG_RETURN_BEHAVIOR_BLOCKING
 *      callbackFxn                 = NULL
 *      timeout                     = SemaphoreP_WAIT_FOREVER
 *      custom                      = NULL
 */
void TRNG_Params_init(TRNG_Params *params);

/*!
 *  @brief  This function opens a given TRNG peripheral.
 *
 *  @pre    TRNG controller has been initialized using TRNG_init()
 *
 *  @param  index         Logical peripheral number for the TRNG indexed into
 *                        the TRNG_config table
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values.
 *
 *  @return A TRNG_Handle on success or a NULL on an error or if it has been
 *          opened already.
 *
 *  @sa     TRNG_init()
 *  @sa     TRNG_close()
 */
TRNG_Handle TRNG_open(uint_least8_t index, TRNG_Params *params);

/*!
 *  @brief  Function to close a TRNG peripheral specified by the TRNG handle
 *
 *  @pre    TRNG_open() has to be called first.
 *
 *  @param  handle A TRNG handle returned from TRNG_open()
 *
 *  @sa     TRNG_open()
 */
void TRNG_close(TRNG_Handle handle);

/*!
 *  @brief  Function performs implementation specific features on a given
 *          TRNG_Handle.
 *
 *  Commands for TRNG_control can originate from TRNG.h or from implementation
 *  specific TRNG*.h (_TRNGCC26XX.h_, _TRNGMSP432.h_, etc.. ) files.
 *  While commands from TRNG.h are API portable across driver implementations,
 *  not all implementations may support all these commands.
 *  Conversely, commands from driver implementation specific TRNG*.h files add
 *  unique driver capabilities but are not API portable across all TRNG driver
 *  implementations.
 *
 *  Commands supported by TRNG.h follow a TRNG_CMD_\<cmd\> naming
 *  convention.<br>
 *  Commands supported by TRNG*.h follow a TRNG*_CMD_\<cmd\> naming
 *  convention.<br>
 *  Each control command defines @b arg differently. The types of @b arg are
 *  documented with each command.
 *
 *  See @ref TRNG_CMD "TRNG_control command codes" for command codes.
 *
 *  See @ref TRNG_STATUS "TRNG_control return status codes" for status codes.
 *
 *  @pre    TRNG_open() has to be called first.
 *
 *  @param  handle      A TRNG handle returned from TRNG_open()
 *
 *  @param  cmd         TRNG.h or TRNG*.h commands.
 *
 *  @param  args        An optional R/W (read/write) command argument
 *                      accompanied with cmd
 *
 *  @return Implementation specific return codes. Negative values indicate
 *          unsuccessful operations.
 *
 *  @sa     TRNG_open()
 */
int_fast16_t TRNG_control(TRNG_Handle handle, uint32_t cmd, void *args);

/*!
 *  @brief  Generate a random number smaller than a number
 *
 *  Generates a random bitstream of the size defined in the \c entropy
 *  CryptoKey in the range 0 <= \c entropy buffer < \c upperBound.
 *  The entropy will be generated and stored according to the storage requirements
 *  defined in the CryptoKey.
 *
 *  \c upperBound must have the same length as defined in \c entropy.
 *
 *  @pre    TRNG_open() has to be called first.
 *
 *  @param  handle A TRNG handle returned from TRNG_open().
 *
 *  @param  entropy A blank, initialized CryptoKey describing the target location
 *                  the entropy shall be stored in.
 *
 *  @param  upperBound The uppper bound of numbers returned, exclusive.
 */
int_fast16_t TRNG_generateEntropyLessThan(TRNG_Handle handle, CryptoKey *entropy, const uint8_t *upperBound);

/*!
 *  @brief  Generate a random number smaller than a number but greater than 0
 *
 *  Generates a random bitstream of the size defined in the \c entropy
 *  CryptoKey in the range 0 < \c entropy buffer < \c upperBound.
 *  The entropy will be generated and stored according to the storage requirements
 *  defined in the CryptoKey.
 *
 *  \c upperBound must have the same length as defined in \c entropy.
 *
 *  @pre    TRNG_open() has to be called first.
 *
 *  @param  handle A TRNG handle returned from TRNG_open().
 *
 *  @param  entropy A blank, initialized CryptoKey describing the target location
 *                  the entropy shall be stored in.
 *
 *  @param  upperBound The uppper bound of numbers returned, exclusive.
 */
int_fast16_t TRNG_generateEntropyNonZeroLessThan(TRNG_Handle handle, CryptoKey *entropy, const uint8_t *upperBound);

/*!
 *  @brief  Generate a random number
 *
 *  Generates a random bitstream of the size defined in the \c entropy
 *  CryptoKey in the range 0 <= \c entropy buffer < 2 ^ (entropy length * 8).
 *  The entropy will be generated and stored according to the storage requirements
 *  defined in the CryptoKey.
 *
 *  @pre    TRNG_open() has to be called first.
 *
 *  @param  handle A TRNG handle returned from TRNG_open().
 *
 *  @param  entropy A blank, initialized CryptoKey describing the target location
 *                  the entropy shall be stored in.
 */
int_fast16_t TRNG_generateEntropy(TRNG_Handle handle, CryptoKey *entropy);




#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_TRNG__include */
