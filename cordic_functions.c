/*******************************************************************************
* File Name:   cordic_functions.c
*
* Description: This file contains function to print the main menu with the list
* operations and get the user selection. This file also contains functions,
* MACROS and variables to get input data from the user, perform the selected
* operations using CORDIC and the software library and print the results.
*
* Related Document: See README.md
*
*
********************************************************************************
* (c) 2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "stdio.h"
#include "cybsp.h"
#include "math.h"
#include "arm_math.h"
#include "cy_retarget_io.h"
#include "cordic_functions.h"

/******************************************************************************
* Macros
*******************************************************************************/
/* Available CORDIC Operations. */
typedef enum Ifx_CORDIC_functions
{
    Ifx_CORDIC_PARK_TRANS   = 0,
    Ifx_CORDIC_SINE         = 1,
    Ifx_CORDIC_COSINE       = 2,
    Ifx_CORDIC_TAN          = 3,
    Ifx_CORDIC_ARC_TAN      = 4,
    Ifx_CORDIC_HYP_SINE     = 5,
    Ifx_CORDIC_HYP_COSINE   = 6,
    Ifx_CORDIC_HYP_TAN      = 7,
    Ifx_CORDIC_HYP_ARC_TAN  = 8,
    Ifx_CORDIC_SQRT         = 9
}Ifx_CORDIC_functions;

/* Multiplier for Q format conversion */
#define Q31_MULTIPLIER (2147483648L) /* 1<<31 */
#define Q30_MULTIPLIER (1073741824L) /* 1<<30 */
#define Q23_MULTIPLIER (8388607L)    /* 1<<23 */
#define Q22_MULTIPLIER (4194304L)    /* 1<<22 */
#define Q15_MULTIPLIER (32768L)      /* 1<<15 */
#define Q11_MULTIPLIER (2048L)       /* 1<<11 */
#define Q8_MULTIPLIER  (256L)        /* 1<<08 */

/* Macros for the conversion of formats */
#define DEG_RAD_MULTIPLIER        (3.141592654/180)
#define RAD_DEG_MULTIPLIER        (180/3.141592654)
#define FLOAT_DEG_TO_RAD(x)       (x * DEG_RAD_MULTIPLIER)
#define FLOAT_RAD_TO_DEG(x)       (x * RAD_DEG_MULTIPLIER)
#define FLOAT_DEG_TO_RAD_Q31(x)   ((CY_CORDIC_Q31_t)(x * (Q31_MULTIPLIER/180)))
#define FLOAT_TO_Q31(x)           ((CY_CORDIC_Q31_t)(x * Q31_MULTIPLIER))
#define FLOAT_TO_Q8_23(x)         ((CY_CORDIC_8Q23_t)(x * Q8_MULTIPLIER))

#define Q31_TO_FLOAT(x)     ((float32_t)(x) / Q31_MULTIPLIER)
#define Q1_30_TO_FLOAT(x)   ((float32_t)(x) / Q30_MULTIPLIER)
#define Q23_TO_FLOAT(x)     ((float32_t)(x) / Q23_MULTIPLIER)
#define Q20_11_TO_FLOAT(x)  ((float32_t)(x) / Q11_MULTIPLIER)
#define Q31_TO_DEG_FLOAT(x) ((float32_t)(x) * (3.141592654 / Q31_MULTIPLIER))

#define CORDIC_CIRCULAR_GAIN (1.646760258121)

/* Input minimum/maximum values. */
#define IN_PARK_ANGLE_MAX      (90)
#define IN_PARK_ANGLE_MIN      (-90)
#define IN_SIN_COS_MAX         (90)
#define IN_SIN_COS_MIN         (-90)
#define IN_TAN_MAX             (89)
#define IN_TAN_MIN             (-89)
#define IN_ATAN_MAX            (57)
#define IN_ATAN_MIN            (-57)
#define IN_HYP_SIN_COS_TAN_MAX (60)
#define IN_HYP_SIN_COS_TAN_MIN (-60)
#define IN_ATANH_MAX           (0.8)
#define IN_ATANH_MIN           (-0.8)

#define ATAN_TANH_IN_SCALING   (127.99)
/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Variable for reading strings from user. */
int8_t               read_string[128] = {0};
int32_t              read_status      = 0;

/* Variable for reading CORDIC function from user */
Ifx_CORDIC_functions cordic_function  = Ifx_CORDIC_PARK_TRANS;

/* Variables for CORDIC operations */
float32_t            angle_deg        = 0;
float32_t            ialpha           = 0;
float32_t            ibeta            = 0;
float32_t            angle_rad        = 0;
float32_t            numerator        = 0;
float32_t            denominator      = 0;

/* Intermediate and results */
float64_t            result_doub      = 0;
CY_CORDIC_Q31_t      result_q31       = 0;
CY_CORDIC_1Q30_t     result_1q30      = 0;
CY_CORDIC_20Q11_t    result_20q11     = 0;
CY_CORDIC_Q31_t      angle_q31        = 0;
CY_CORDIC_8Q23_t     numerator_8q23   = 0;
CY_CORDIC_8Q23_t     denominator_8q23 = 0;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void park_transform();
void sine();
void cosine();
void tangent();
void arc_tangent();
void hyperbolic_sine();
void hyperbolic_cosine();
void hyperbolic_tangent();
void hyperbolic_arc_tangent();
void square_root();
cy_en_cordic_status_t check_range(float32_t low_limit,
                                  float32_t high_limit,
                                  float32_t number);

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: run_cordic_functions
*********************************************************************************
* Summary:
* This function prints the main menu and gets the required operation to be
* performed form the user. Based on the selected operation, the required function
* will be called and the operation will be performed.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void run_cordic_functions()
{
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("************************************************************\r\n");
    printf("PSOC Control C3M/P8: CORDIC\r\n");
    printf("************************************************************\r\n\n");

    for (;;)
    {
        /* Main menu. To select the required operation. */
        printf("Please select the required operation from the list. \r\n");
        printf("0 - park transform \r\n");
        printf("1 - sine \r\n");
        printf("2 - cosine \r\n");
        printf("3 - tangent \r\n");
        printf("4 - arc tangent \r\n");
        printf("5 - hyperbolic sine \r\n");
        printf("6 - hyperbolic cosine \r\n");
        printf("7 - hyperbolic tangent \r\n");
        printf("8 - hyperbolic arc tangent \r\n");
        printf("9 - square root \r\n");
        printf(">> \r\n");

        read_status = scanf("%120s", read_string);

        if(0 < read_status)
        {
            cordic_function = (Ifx_CORDIC_functions)atoi((const char *)read_string);

            /* Execution of the selected operation. */
            switch(cordic_function)
            {
            case Ifx_CORDIC_PARK_TRANS:
            {
                park_transform(); /* Park transform function */
            }
            break;

            case Ifx_CORDIC_SINE:
            {
                sine(); /* Sine function */
            }
            break;

            case Ifx_CORDIC_COSINE:
            {
                cosine();  /* Cosine function */
            }
            break;

            case Ifx_CORDIC_TAN:
            {
                tangent(); /* Tangent function */
            }
            break;

            case Ifx_CORDIC_ARC_TAN:
            {
                arc_tangent(); /* Arc tangent function */
            }
            break;

            case Ifx_CORDIC_HYP_SINE:
            {
                hyperbolic_sine(); /* Hyperbolic sine function */
            }
            break;

            case Ifx_CORDIC_HYP_COSINE:
            {
                hyperbolic_cosine(); /* Hyperbolic cosine function */
            }
            break;

            case Ifx_CORDIC_HYP_TAN:
            {
                hyperbolic_tangent(); /* Hyperbolic tangent function */
            }
            break;

            case Ifx_CORDIC_HYP_ARC_TAN:
            {
                hyperbolic_arc_tangent(); /* Hyperbolic arc tangent function */
            }
            break;

            case Ifx_CORDIC_SQRT:
            {
                square_root(); /* Square root function */
            }
            break;

            default:
            {
                /* A value which is not present in the list is entered. */
                printf("Wrong option selected. Please try again... \r\n");
            }
            break;
            }
        }
        printf("\r\n\r\n");
    }
}

/*******************************************************************************
* Function Name: check_range
*********************************************************************************
* Summary:
* This is the function for checking the entered value is in the expected range.
*
* Parameters:
* float32_t low_limit  Lowest value accepted
* float32_t high_limit Highest value accepted
* float32_t value      The value to be checked against the range
*
* Return:
* cy_en_cordic_status_t    Validation status
*
*******************************************************************************/
cy_en_cordic_status_t check_range(float32_t low_limit,
                                  float32_t high_limit,
                                  float32_t value)
{
    cy_en_cordic_status_t return_val = CY_CORDIC_SUCCESS;

    if((low_limit > value) || (high_limit < value))
    {
        printf("\r\nEntered number is not in range. \r\n");

        return_val = CY_CORDIC_BAD_PARAM;
    }

    return return_val;
}

/*******************************************************************************
* Function Name: park_transform
*********************************************************************************
* Summary:
* This is the function for calculating the park transform. It reads the angle,
* alpha and beta values from the user and calculates the park transform using
* CORDIC and math library. Then it prints the results.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void park_transform()
{
    printf("\r\nSelected option - park transform.");

    /* Getting angle for park transform from user */
    printf("\r\nEnter angle in degree (between -90 and 90): \r\n");
    read_status = scanf("%120s", read_string);

    /* Checking input read status */
    if(0 < read_status)
    {
        /* Converting to float */
        angle_deg = atof((const char *)read_string);

        /* Checking read data range */
        if(CY_CORDIC_SUCCESS == check_range(IN_PARK_ANGLE_MIN, IN_PARK_ANGLE_MAX, angle_deg))
        {
            /* Getting alpha value for the park transform from user */
            printf("\r\nEnter i alpha (between -1 and 1): \r\n");
            read_status = scanf("%120s", read_string);

            /* Checking input read status */
            if(0 < read_status)
            {
                /* Converting to float */
                ialpha = atof((const char *)read_string);

                /* Checking read data range */
                if(CY_CORDIC_SUCCESS == check_range(-1, 1, ialpha))
                {
                    /* Getting beta value for the park transform from user */
                    printf("\r\nEnter i beta (between -1 and 1): \r\n");
                    read_status = scanf("%120s", read_string);

                    /* Checking input read status */
                    if(0 < read_status)
                    {
                        /* Converting to float */
                        ibeta = atof((const char *)read_string);

                        /* Checking read data range */
                        if(CY_CORDIC_SUCCESS == check_range(-1, 1, ibeta))
                        {
                            CY_CORDIC_Q31_t p_id    = 0;
                            CY_CORDIC_Q31_t p_iq    = 0;
                            CY_CORDIC_Q31_t sin_q31 = 0;
                            CY_CORDIC_Q31_t cos_q31 = 0;
                            float32_t       sin_of_angle = 0;
                            float32_t       cos_of_angle = 0;
                            int32_t         i_alpha_q31 = 0;
                            int32_t         i_beta_q31 = 0;
                            float32_t       result_p_id = 0;
                            float32_t       result_p_iq = 0;
                            cy_stc_cordic_parkTransform_result_t park_result;

                            /* Converting the angle in degree to radian and radian in Q31 format */
                            angle_rad = FLOAT_DEG_TO_RAD(angle_deg);
                            angle_q31 = FLOAT_DEG_TO_RAD_Q31(angle_deg);

                            /* Converting read alpha and beta to Q31 format */
                            i_alpha_q31 = FLOAT_TO_Q31(ialpha);
                            i_beta_q31  = FLOAT_TO_Q31(ibeta);

                            /* Starting park transform using CORDIC */
                            Cy_CORDIC_ParkTransformNB(PPCA_MXCORDIC0,
                                                      angle_q31,
                                                      i_alpha_q31,
                                                      i_beta_q31);

                            /* Waiting for park transform completion */
                            while(Cy_CORDIC_IsBusy(PPCA_MXCORDIC0)){}

                            /* Getting park transform result from CORDIC */
                            Cy_CORDIC_GetParkResult(PPCA_MXCORDIC0,
                                                    &park_result);

                            /* Converting results from Q23 to float */
                            result_p_id = Q23_TO_FLOAT(park_result.parkTransformId);
                            result_p_iq = Q23_TO_FLOAT(park_result.parkTransformIq);

                            /* Removing the gain from CORDIC circular function */
                            result_p_id = result_p_id * (1/CORDIC_CIRCULAR_GAIN);
                            result_p_iq = result_p_iq * (1/CORDIC_CIRCULAR_GAIN);

                            printf("\r\nPark transform using CORDIC. Id: %f. Iq: %f.", result_p_id, result_p_iq);

                            /* Calculating sin and cos of the angle required by the math library park transform function */
                            sin_of_angle = sin((float64_t)angle_rad);
                            cos_of_angle = cos((float64_t)angle_rad);

                            /* Converting sin and cos of the angle to the q31 format */
                            sin_q31 = FLOAT_TO_Q31(sin_of_angle);
                            cos_q31 = FLOAT_TO_Q31(cos_of_angle);

                            /* Calculating park transform using software */
                            arm_park_q31(i_alpha_q31,
                                         i_beta_q31,
                                         &p_id,
                                         &p_iq,
                                         sin_q31,
                                         cos_q31);

                            /* Converting results from q23 to float */
                            result_p_id = Q31_TO_FLOAT(p_id);
                            result_p_iq = Q31_TO_FLOAT(p_iq);

                            printf("\r\nPark transform using math library. Id: %f Iq: %f.\r\n", result_p_id, result_p_iq);
                        }
                    }
                }
            }
        }
    }
}

/*******************************************************************************
 * Function Name: sine
 *********************************************************************************
 * Summary:
 * This is the function for calculating the sine. It reads the angle from the
 * user and calculates the sine using CORDIC and math library functions. Then
 * it prints the results.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void sine ()
{
    printf("\r\nSelected option - sine.");

    /* Getting angle for sine calculation from user */
    printf("\r\nEnter the angle in degree(between -90 and 90): \r\n");

    /* Reading angle from user */
    read_status = scanf("%120s", read_string);

    /* Checking input read status */
    if(0 < read_status)
    {
        /* Converting to float */
        angle_deg = atof((const char *)read_string);

        /* Checking read data range */
        if(CY_CORDIC_SUCCESS == check_range(IN_SIN_COS_MIN, IN_SIN_COS_MAX, angle_deg))
        {
            /* Converting the angle in degree to radian and radian in Q31 format */
            angle_rad = FLOAT_DEG_TO_RAD(angle_deg);
            angle_q31 = FLOAT_DEG_TO_RAD_Q31(angle_deg);

            /* Calculating sine using CORDIC */
            result_q31 = Cy_CORDIC_Sin(PPCA_MXCORDIC0,
                                       angle_q31);

            /* Converting the result in Q31 format to float */
            result_doub = Q31_TO_FLOAT(result_q31);

            printf("\r\nSine of the angle using CORDIC: %f.", result_doub);

            /* Calculating sine using software */
            result_doub = sin((float64_t)angle_rad);

            printf("\r\nSine of the angle using math library: %f.\r\n", result_doub);
        }
    }
}

/*******************************************************************************
 * Function Name: cosine
 *********************************************************************************
 * Summary:
 * This is the function for calculating the cosine. It reads the angle from the
 * user and calculates the cosine using CORDIC and math library functions. Then
 * it prints the results.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void cosine()
{
    printf("\r\nSelected option - cosine.");

    /* Getting angle for cosine calculation from user */
    printf("\r\nEnter the angle in degree(between -90 and 90): \r\n");

    /* Reading angle from user */
    read_status = scanf("%120s", read_string);

    /* Checking input read status */
    if(0 < read_status)
    {
        /* Converting to float */
        angle_deg = atof((const char *)read_string);

        /* Checking read data range */
        if(CY_CORDIC_SUCCESS == check_range(IN_SIN_COS_MIN, IN_SIN_COS_MAX, angle_deg))
        {
            /* Converting the angle in degree to radian and radian in Q31 format */
            angle_rad = FLOAT_DEG_TO_RAD(angle_deg);
            angle_q31 = FLOAT_DEG_TO_RAD_Q31(angle_deg);

            /* Calculating cosine using CORDIC */
            result_q31 = Cy_CORDIC_Cos(PPCA_MXCORDIC0,
                                       angle_q31);

            /* Converting the result in Q31 format to float */
            result_doub = Q31_TO_FLOAT(result_q31);

            printf("\r\nCosine of the angle using CORDIC: %f.", result_doub);

            /* Calculating cosine using software */
            result_doub = cos((float64_t)angle_rad);

            printf("\r\nCosine of the angle using math library: %f.\r\n", result_doub);
        }
    }
}

/*******************************************************************************
 * Function Name: tangent
 *********************************************************************************
 * Summary:
 * This is the function for calculating the tangent. It reads the angle from the
 * user and calculates the tangent using CORDIC and math library function. Then
 * it prints the results.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void tangent()
{
    printf("\r\nSelected option - tangent.");

    /* Getting angle for tangent calculation from user */
    printf("\r\nEnter the angle in degree (between -89 and 89): \r\n");

    /* Reading angle from user */
    read_status = scanf("%120s", read_string);

    /* Checking input read status */
    if(0 < read_status)
    {
       /* Converting to float */
       angle_deg = atof((const char *)read_string);

        /* Checking read data range */
        if(CY_CORDIC_SUCCESS == check_range(IN_TAN_MIN, IN_TAN_MAX, angle_deg))
        {
            /* Converting the angle in degree to radian and radian in Q31 format */
            angle_rad = FLOAT_DEG_TO_RAD(angle_deg);
            angle_q31 = FLOAT_DEG_TO_RAD_Q31(angle_deg);

            /* Calculating tangent using CORDIC */
            result_20q11 = Cy_CORDIC_Tan(PPCA_MXCORDIC0,
                                         angle_q31);

            /* Converting the result in 20Q11 format to float */
            result_doub = Q20_11_TO_FLOAT(result_20q11);

            printf("\r\nTangent of the angle using CORDIC: %f.", result_doub);

            /* Calculating tangent using software */
            result_doub = tan((float64_t)angle_rad);

            printf("\r\nTangent of the angle using math library: %f.\r\n", result_doub);
        }
    }
}

/*******************************************************************************
 * Function Name: arc_tangent
 *********************************************************************************
 * Summary:
 * This is the function for calculating the arc tangent. It reads the numerator
 * and denominator values from the user and calculates the arc tangent using CORDIC
 * and math library functions. Then it prints the results.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void arc_tangent()
{
    printf("\r\nSelected option - arc tangent.");

    /* Getting numerator and denominator values for arc tan calculation from user */
    printf("\r\nEnter the value(between -57 and 57): \r\n");

    /* Reading numerator from user */
    read_status = scanf("%120s", read_string);

    /* Checking input read status */
    if(0 < read_status)
    {
        /* Converting to float */
        numerator = atof((const char *)read_string);

        /* Checking read data range */
        if(CY_CORDIC_SUCCESS == check_range(IN_ATAN_MIN, IN_ATAN_MAX, numerator))
        {
            numerator = numerator * ATAN_TANH_IN_SCALING;
            denominator = ATAN_TANH_IN_SCALING;

            /* Converting the numbers to Q8_23 format */
            numerator_8q23   = FLOAT_TO_Q8_23(numerator);
            denominator_8q23 = FLOAT_TO_Q8_23(denominator);

            /* Calculating arc tangent using CORDIC */
            result_q31 = Cy_CORDIC_ArcTan(PPCA_MXCORDIC0,
                                          denominator_8q23,
                                          numerator_8q23);

            /* Converting the result in Q31 format to float */
            result_doub = Q31_TO_DEG_FLOAT(result_q31);

            /* Converting the returned angle from radian to degree */
            result_doub = FLOAT_RAD_TO_DEG(result_doub);

            printf("\r\nArcTan in degree using CORDIC: %f.", result_doub);

            /* Calculating arc tangent using software */
            result_doub = atan2((float64_t)numerator,
                                (float64_t)denominator);

            /* Converting the returned angle from radian to degree */
            result_doub = FLOAT_RAD_TO_DEG(result_doub);

            printf("\r\nArcTan in degree using math library: %f.\r\n", result_doub);
        }
    }
}

/*******************************************************************************
 * Function Name: hyperbolic_sine
 *********************************************************************************
 * Summary:
 * This is the function for calculating the hyperbolic sine. It reads the angle
 * from the user and calculates the hyperbolic sine using CORDIC and math library
 * function. Then it prints the results.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void hyperbolic_sine()
{
    printf("\r\nSelected option - hyperbolic sine.");

    /* Getting angle for hyperbolic sine calculation from user */
    printf("\r\nEnter the angle in degree (between -60 and 60): \r\n");

    /* Reading angle from user */
    read_status = scanf("%120s", read_string);

    /* Checking input read status */
    if(0 < read_status)
    {
        /* Converting to float */
        angle_deg = atof((const char *)read_string);

        /* Checking read data range */
        if(CY_CORDIC_SUCCESS == check_range(IN_HYP_SIN_COS_TAN_MIN, IN_HYP_SIN_COS_TAN_MAX, angle_deg))
        {
            /* Converting the angle in degree to radian and radian in Q31 format */
            angle_rad = FLOAT_DEG_TO_RAD(angle_deg);
            angle_q31 = FLOAT_DEG_TO_RAD_Q31(angle_deg);

            /* Calculating hyperbolic sine using CORDIC */
            result_1q30 = Cy_CORDIC_Sinh(PPCA_MXCORDIC0,
                                         angle_q31);

            /* Converting the result in 1Q30 format to float */
            result_doub = Q1_30_TO_FLOAT(result_1q30);

            printf("\r\nHyperbolic Sine using CORDIC: %f.", result_doub);

            /* Calculating hyperbolic sine using software */
            result_doub = sinh((float64_t)angle_rad);

            printf("\r\nHyperbolic Sine using math library: %f.\r\n", result_doub);
        }
    }
}

/*******************************************************************************
 * Function Name: hyperbolic_cosine
 *********************************************************************************
 * Summary:
 * This is the function for calculating the hyperbolic cosine. It reads the angle
 * from the user and calculates the hyperbolic cosine using CORDIC and math library
 * functions. Then it prints the results.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void hyperbolic_cosine()
{
    printf("\r\nSelected option - hyperbolic cosine.");

    /* Getting angle for hyperbolic cosine calculation from user */
    printf("\r\nEnter the angle in degree (between -60 and 60): \r\n");

    /* Reading angle from user */
    read_status = scanf("%120s", read_string);

    /* Checking input read status */
    if(0 < read_status)
    {
        /* Converting to float */
        angle_deg = atof((const char *)read_string);

        /* Checking read data range */
        if(CY_CORDIC_SUCCESS == check_range(IN_HYP_SIN_COS_TAN_MIN, IN_HYP_SIN_COS_TAN_MAX, angle_deg))
        {
            /* Converting the angle in degree to radian and radian in Q31 format */
            angle_rad = FLOAT_DEG_TO_RAD(angle_deg);
            angle_q31 = FLOAT_DEG_TO_RAD_Q31(angle_deg);

            /* Calculating hyperbolic cosine using CORDIC */
            result_1q30 = Cy_CORDIC_Cosh(PPCA_MXCORDIC0,
                                         angle_q31);

            /* Converting the result in 1Q30 format to float */
            result_doub = Q1_30_TO_FLOAT(result_1q30);

            printf("\r\nHyperbolic Cosine using CORDIC: %f.", result_doub);

            /* Calculating hyperbolic cosine using software */
            result_doub = cosh((float64_t)angle_rad);

            printf("\r\nHyperbolic Cosine using math library: %f.\r\n", result_doub);
        }
    }
}

/*******************************************************************************
 * Function Name: hyperbolic_tangent
 *********************************************************************************
 * Summary:
 * This is the function for calculating the hyperbolic tangent. It reads the
 * angle from the user and calculates the hyperbolic tangent using CORDIC
 * and math library functions. Then it prints the results.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void hyperbolic_tangent()
{
    printf("\r\nSelected option - hyperbolic tangent.");

    /* Getting angle for hyperbolic tangent calculation from user */
    printf("\r\nEnter the angle in degree (between -60 and 60): \r\n");

    /* Reading angle from user */
    read_status = scanf("%120s", read_string);

    /* Checking input read status */
    if(0 < read_status)
    {
        /* Converting to float */
        angle_deg = atof((const char *)read_string);

        /* Checking read data range */
        if(CY_CORDIC_SUCCESS == check_range(IN_HYP_SIN_COS_TAN_MIN, IN_HYP_SIN_COS_TAN_MAX, angle_deg))
        {
            /* Converting the angle in degree to radian and radian in Q31 format */
            angle_rad = FLOAT_DEG_TO_RAD(angle_deg);
            angle_q31 = FLOAT_DEG_TO_RAD_Q31(angle_deg);

            /* Calculating hyperbolic tangent using CORDIC */
            result_20q11 = Cy_CORDIC_Tanh(PPCA_MXCORDIC0,
                                          angle_q31);

            /* Converting the result in 20Q11 format to float */
            result_doub = Q20_11_TO_FLOAT(result_20q11);

            printf("\r\nHyperbolic Tangent using CORDIC: %f.", result_doub);

            /* Calculating hyperbolic tangent using software */
            result_doub = tanh((float64_t)angle_rad);

            printf("\r\nHyperbolic Tangent using math library: %f.\r\n", result_doub);
        }
    }
}

/*******************************************************************************
 * Function Name: hyperbolic_arc_tangent
 *********************************************************************************
 * Summary:
 * This is the function for calculating the hyperbolic arc tangent. It reads
 * numerator and denominator numbers from the user and calculates the hyperbolic
 * arc tangent using CORDIC. Then it prints the results.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void hyperbolic_arc_tangent()
{
    float32_t read_value = 0;

    printf("\r\nSelected option - hyperbolic arc tangent.");

    /* getting value for hyperbolic arc tan calculation from user */
    printf("\r\nEnter the value(between -0.8 and 0.8): \r\n");

    /* reading numerator from user */
    read_status = scanf("%120s", read_string);

    /* Checking input read status */
    if(0 < read_status)
    {
        /* Converting to float */
        read_value = atof((const char *)read_string);

        /* checking read data range */
        if(CY_CORDIC_SUCCESS == check_range(IN_ATANH_MIN, IN_ATANH_MAX, read_value))
        {
            numerator = read_value * ATAN_TANH_IN_SCALING;
            denominator = ATAN_TANH_IN_SCALING;

            /* Converting the numbers to 8Q23 format */
            numerator_8q23   = FLOAT_TO_Q8_23(numerator);
            denominator_8q23 = FLOAT_TO_Q8_23(denominator);

            /* Calculating hyperbolic arc tangent using CORDIC */
            result_q31 = Cy_CORDIC_ArcTanh(PPCA_MXCORDIC0,
                                           denominator_8q23,
                                           numerator_8q23);

            /* Converting the result in q31 format to float */
            result_doub = Q31_TO_DEG_FLOAT(result_q31);

            /* Converting the returned angle from radian to degree */
            result_doub = FLOAT_RAD_TO_DEG(result_doub);

            printf("\r\nHyperbolic ArcTan in degree using CORDIC: %f.", result_doub);

            /* Calculating hyperbolic arc tangent using software */
            result_doub = atanh(read_value);

            /* Converting the returned angle from radian to degree */
            result_doub = FLOAT_RAD_TO_DEG(result_doub);

            printf("\r\nHyperbolic ArcTan in degree using math library: %f.\r\n", result_doub);
        }
    }
}

/*******************************************************************************
 * Function Name: square_root
 *********************************************************************************
 * Summary:
 * This is the function for calculating the square root. It reads a number from
 * the user and calculates the square root using CORDIC and math library functions.
 * Then it prints the results.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void square_root()
{
    float32_t number          = 0;
    int32_t   number_q31      = 0;
    uint32_t  square_root_q31 = 0;
    printf("\r\nSelected option - square root.");

    /* Getting number for square root calculation from user */
    printf("\r\nEnter the value above 0 and below 1: \r\n");

    /* Reading the number from user */
    read_status = scanf("%120s", read_string);
    if(0 < read_status)
    {
        /* Converting to float */
        number = atof((const char *)read_string);

        /* Checking read data range */
        if((CY_CORDIC_SUCCESS == check_range(0, 1, number)) && (0 != number))
        {
            /* converting the number to Q31 format */
            number_q31 = FLOAT_TO_Q31(number);

            /* Calculating square root using CORDIC */
            square_root_q31 = Cy_CORDIC_Sqrt(PPCA_MXCORDIC0,
                                             number_q31);

            /* Converting the result in Q31 format to float */
            result_doub = Q31_TO_FLOAT(square_root_q31);

            printf("\r\nSquare root using CORDIC: %f.", result_doub);

            /* Calculating square root using software */
            result_doub = sqrt((float64_t)number);

            printf("\r\nSquare root using math library: %f. \r\n", result_doub);
        }
        if(0 == number)
        {
            printf("\r\nEntered number is 0. \r\n");
        }
    }
}
/* [] END OF FILE */
