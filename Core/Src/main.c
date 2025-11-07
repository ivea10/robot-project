/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
uint8_t demarrage = 0;

int T_batt = 0;
int Vbatt = 0;

int seuilBat = 3723;     // 90% de 4096 (valeur max)
int seuilDistance = 335;

uint32_t start_time, end_time, time_elapsed;
float distance = 0;

const int positionMin = 300;
const int positionMax = 8000;

int vitesseMax = 14286;  // vitesse de 20cm/s

int i = positionMin;
int angle = 0;
int pas = 50;

uint8_t estAligne = 0;
uint8_t premiereDetection = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void avancer();
void arret();
void reculer();
void rotation();
bool objetDetecte();
void recherche();
void alignement(int n);
void delay_us(uint16_t us);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, positionMin);

  demarrage = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */

    // Gestion de la batterie
    if (T_batt > 50) {
      T_batt = 0;
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
      Vbatt = HAL_ADC_GetValue(&hadc1);
    }

    if (Vbatt < seuilBat) {
      HAL_GPIO_WritePin(Alert_batt_GPIO_Port, Alert_batt_Pin, 1);
    } else {
      HAL_GPIO_WritePin(Alert_batt_GPIO_Port, Alert_batt_Pin, 0);
    }

    if (demarrage)
    {
      // Gestion du sonar
      HAL_GPIO_WritePin(TRIG_SONAR_GPIO_Port, TRIG_SONAR_Pin, GPIO_PIN_SET);
      delay_us(100);
      HAL_GPIO_WritePin(TRIG_SONAR_GPIO_Port, TRIG_SONAR_Pin, GPIO_PIN_RESET);

      uint32_t start = 0, stop = 0, duration = 0;
      uint32_t timeout_start = __HAL_TIM_GET_COUNTER(&htim1);
      uint8_t timeout_flag = 0;

      // Attente de ECHO_SONAR = 0 ou timeout
      while (HAL_GPIO_ReadPin(ECHO_SONAR_GPIO_Port, ECHO_SONAR_Pin) == GPIO_PIN_RESET) {
        if ( (__HAL_TIM_GET_COUNTER(&htim1) - timeout_start) > 50000 ) {
          timeout_flag = 1;
          break;
        }
      }

      if (!timeout_flag) {
        start = __HAL_TIM_GET_COUNTER(&htim1);
      }

      // Attente de ECHO_SONAR = 1 ou timeout
      while (HAL_GPIO_ReadPin(ECHO_SONAR_GPIO_Port, ECHO_SONAR_Pin) == GPIO_PIN_SET) {
        if ( (__HAL_TIM_GET_COUNTER(&htim1) - start) > 100000 ) {
          timeout_flag = 1;
          break;
        }
      }

      // Gestion du sonar
      if (!timeout_flag) {
        stop = __HAL_TIM_GET_COUNTER(&htim1);
        if (timeout_flag) {
          // pas d'obstacle ou hors limite
          distance = 10000;
        } else {
          duration = stop - start;
          distance = (duration * 0.0343f) / 2.0f;
        }
      }

      // Cas de détection
      if (objetDetecte()) {
        premiereDetection = 1;
        if (!estAligne) {
          alignement(i);
          estAligne = 1;
        } else if (estAligne) {
          if (distance > 67) {
            avancer();
          } else {
            arret();
          }
        }
      }
      else if (!objetDetecte()) {
        arret();
        if (premiereDetection == 1) {
          HAL_Delay(100);
          if (!objetDetecte()) {
            premiereDetection = 0;
            estAligne = 0;
            i = positionMax/2;
            recherche();
          }
        } else if (premiereDetection == 0) {
          recherche();
        }
      }

      angle = i * 180 / positionMax;
      HAL_Delay(50);
    }
    /* USER CODE END 3 */
  }
  /* USER CODE END WHILE */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) T_batt += 1;
}

void avancer() {
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, vitesseMax);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, vitesseMax);

  HAL_GPIO_WritePin(Cmde_DirD_GPIO_Port, Cmde_DirD_Pin, 1);
  HAL_GPIO_WritePin(Cmde_DirG_GPIO_Port, Cmde_DirG_Pin, 1);
}

void arret() {
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

void reculer() {
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, vitesseMax);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, vitesseMax);

  HAL_GPIO_WritePin(Cmde_DirD_GPIO_Port, Cmde_DirD_Pin, 0);
  HAL_GPIO_WritePin(Cmde_DirG_GPIO_Port, Cmde_DirG_Pin, 0);
}

void rotation(int angle) {
  // 0 <= angle <= 180 et 0° correspond à position à droite
  int sens = 0;   // 0 = droite ; 1 = gauche
  if (angle < 90) {
    angle = 90 - angle;
    sens = 0;
  } else {
    angle -= 90;
    sens = 1;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, vitesseMax);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, vitesseMax);

  HAL_GPIO_WritePin(Cmde_DirD_GPIO_Port, Cmde_DirD_Pin, sens);
  HAL_GPIO_WritePin(Cmde_DirG_GPIO_Port, Cmde_DirG_Pin, 1 - sens);
  HAL_Delay(angle * 1750 / 90); // valeur trouvée empiriquement
  arret();
}

void recherche() {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, i);
  HAL_Delay(50);
  i += pas;
  if (i > positionMax) pas *= -1;
  if (i < positionMin) pas *= -1;
}

void alignement(int n) {
  int angle = n * 180 / positionMax;
  rotation(angle);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, positionMax/2);
}

bool objetDetecte() {
  if (distance < seuilDistance) {
    return true;
  }
  return false;
}

void delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == startButton_Pin) {
    demarrage = 1 - demarrage;
  }
}
/* USER CODE END 4 */