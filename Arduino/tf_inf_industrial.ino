// importa las librerias para usar la pantalla oled
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// Configura la pantalla oled
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Valores constantes para el sensor ultrasonico
const int pinDO_TRIG = 5;       // pin para el trigger
const int pinDI_ECO = 6;        // pin para el echo
double nivel;                // valor del nivel de agua
double distancia;            // valor de distancia al sensor
double nivel_ant = 0.0;     // valor del nivel sensado en el ciclo anterior.
const double a = 0.95;      // coeficiente utilizado en el filtro aplicado a la salida del nivel sensado
bool primer_vuelta;

// Generacion de PWM de NIVEL
const int pinDO_NVL = 9;    // pin de salida digital (PWM) para el nivel

// Generacion de PWM de BMB_PERTURBADORA
const int pinDO_PERT = 10;  // pin de salida digital (PWM) para la bomba de perturbacion
const int pinAI_PERT = A2;  // pin de entrada analogica de control de la bomba de perturbacion
int valOutPWM_PERT = 0;     // valor de salida (del PWM) para la bomba perturbacion
int valIn_PERT = 0;         // valor de la entrada de control de la bomba de perturbacion

// Generacion de PWM de BMB_CARGA
const int pinDO_CARG = 11;  // pin de salida digital (PWM) para la bomba de carga
const int pinAI_CARG = A3;  // pin de entrada analogica de control de la bomba de carga
int valOutPWM_CARG = 0;     // valor de salida (del PWM) para la bomba de carga
int valIn_CARG = 0;         // valor de la entrada de control de la bomba de carga

//Boton para calibración de tacho vacio
const int btn_CALB = 12;

// Valores constantes para el dibujo del tacho en la pantalla OLED
const int x_tacho = 5;
const int y_tacho = 15;
const int ancho_tacho = 25;
const int alto_tacho = 40;
const int nivel_tacho_vacio = y_tacho + alto_tacho; // 55
const int nivel_tacho_lleno = y_tacho;              // 15
const int nivel_tacho_actual = nivel_tacho_vacio;   // 55 (arranca el dibujo con el tacho vacio)
int aux_nivel = nivel_tacho_actual;                 // variable auxiliar 

/********************************************************************************************************************
 * Resumen de la configuracion de los pines en el arduino
 * 
 *                  +---------+
 *                  |         |
 *             Vcc -+    A    +- (05)TRIG
 *             GND -+    R    +- (06)ECO
 *                  |    D    +- (12)BTN_CALB
 *    BMB_PERT(A2) -+    U    |
 *    BMB_CARG(A3) -+    I    +- (09)PWM_NVL
 *    SDA_OLED(A4) -+    N    +- (10)PWM_PERT
 *    SCL_OLED(A5) -+    O    +- (11)PWM_CARG
 *                  |         |
 *                  +---------+
 * 
 *******************************************************************************************************************/


/********************************************************************************************************************
 * 
 * Escalado para la bomba: Valor de consigna en 10 bits a PWM con 8 bits
 * 
 ********************************************************************************************************************
 * Relacion entre cuentas de la entrada analogica de las bombas y la correspondiente salida PWM de cada bomba.
 * 
 * (cuentas - PWM)
 * 255 |          /
 *     |        /     
 *     |      /    
 *     |    /          
 *     |  /           
 *     |/              
 *   0 +----------+-
 *                 1023 (cuentas - ADC)
 */

// Valores constantes para el escalado
const int CT_IN_10_H = 1023;
const int CT_IN_10_L = 205;
const int CT_OUT_8_H = 255;
const int CT_OUT_8_L = 0;

// Pendiente de la recta de escalado
const double M_CONV_CT = (double) CT_OUT_8_H / ((double) CT_IN_10_H - (double) CT_IN_10_L) ; 

/********************************************************************************************************************
 * 
 * Escalado para el dibujo del tacho: Nivel de agua en cm a porcentaje de nivel
 * 
 ********************************************************************************************************************
 *
 * (%)
 * 100 |\
 *     |  \           
 *     |    \     
 *     |      \       
 *     |        \      
 *     |          \    
 *   0 +-----------+-
 *                 20 (cm)
 */

// El rango de variacion de distancia (cm) para el nivel
double distancia_h = 22;   // max cm
const double distancia_l = 0;    // min cm


// El rango de variacion del porcentaje (%) para el nivel
int porcentaje_h = 100;       // max %
int porcentaje_l = 0;         // min %

// Pendiente de la recta de escalado
double m_cm_perc = (double) (porcentaje_l - porcentaje_h ) / (double) (distancia_h - distancia_l); // pendiente  

// Funcion para la ecuacion de la recta, devuelve el nivel en porciento dado el nivel en cm
int f_cm_perc(int distancia_sensor) {
  return (m_cm_perc) * ( distancia_sensor - distancia_l ) + 100;
}

/********************************************************************************************************************
 * 
 * Escalado para el dibujo del tacho: Porcentaje de nivel a pixel para el grafico
 * 
 ********************************************************************************************************************
 * Relacion entre el estado del tacho (dado por su porcentaje de nivel) y la representacion grafica 
 * en la pantalla OLED (dado en pixeles)
 * 
 * (px)
 *  60 |\
 *     |  \
 *     |    \
 *     |      \    
 *     |        \      
 *     |          \    
 *     |            \  
 *  20 + - - - - - - -\
 *     |               |
 *     |               |
 *   0 +---------------+-
 *     0              100 (%)
 */

// Pendiente de la recta de escalado
double m_perc_px = (double) (nivel_tacho_lleno - nivel_tacho_vacio) / (double) (porcentaje_h - porcentaje_l);

// Funcion para la ecuacion de la recta, devuelve el nivel en pixeles dado un valor de porcentaje del nivel
int f_perc_px(int nivel_porcentaje) {
  return (m_perc_px) * ( nivel_porcentaje - porcentaje_l ) + nivel_tacho_vacio;
}

/********************************************************************************************************************
 * 
 * Escalado para el PWM de nivel: Nivel en cm a PWM con 8 bits
 * 
 ********************************************************************************************************************
/* Relacion entre el nivel de agua medido en cm y las cuentas para generar el PWM con 8 bits.
 * 
 * (cuentas - PWM)
 * 255 |\
 *     |  \           
 *     |    \     
 *     |      \       
 *     |        \      
 *     |          \    
 *   0 +-----------+-
 *                 20 (cm)
 */

//Pendiente
//const double m_cm_ct8 = (double) (CT_OUT_8_L - CT_OUT_8_H) / (double) (distancia_h - distancia_l);
const double m_cm_ct8 = (double) ( CT_OUT_8_H - CT_OUT_8_L) / (double) (distancia_h - distancia_l); 

// Funcion para la ecuacion de la recta, devuelve las cuentas (8 bit) del PWM que representa el nivel del tacho
/*int f_cm_ct8(double distancia_sensor) {
  return (m_cm_ct8) * ( distancia_sensor - distancia_l ) + CT_OUT_8_H;
}*/

int f_cm_ct8(double nivel){
  return (m_cm_ct8) * nivel;
}



/********************************************************************************************************************
 ********************************************************************************************************************
 *                                                      PROGRAMA
 ********************************************************************************************************************
 *******************************************************************************************************************/

void setup() {
  u8g2.begin();
  u8g2.clearBuffer();                           // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);           // choose a suitable font
  u8g2.drawStr(0,10,"Teclado matricial oled");  // write something to the internal memory
  u8g2.sendBuffer();                            // transfer inte// Generacion de PWM de BMB_PERTURBADORA

  pinMode(pinDO_TRIG, OUTPUT);  // trigger como salida
  pinMode(pinDI_ECO, INPUT);    // echo como entrada
  
  pinMode(pinDO_PERT, OUTPUT);
  pinMode(pinAI_PERT, INPUT);

  pinMode(pinDO_CARG, OUTPUT);
  pinMode(pinAI_CARG, INPUT);

  pinMode(btn_CALB, INPUT_PULLUP);
  
  nivel = 0.0;
  distancia = 0.0;

  primer_vuelta = true;

  Serial.begin(9600);
    
}

void loop() {

  u8g2.clearBuffer();                         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);         // choose a suitable font
  u8g2.drawStr(0,10,"Sist. Control Nivel");   // write something to the internal memory
    
  // Procesamiento de entradas analógicas (del PLC)
  // Entrada BMB_CARGA
  valIn_CARG = analogRead(pinAI_CARG);
  
  // Se genera el PWM de la primera entrada
  valOutPWM_CARG =  M_CONV_CT * (valIn_CARG - CT_IN_10_L);

  //Si el valor (cuentas) aplicado a la bomba de carga es menor a 25, se apaga totalmente
  if ( valOutPWM_CARG < (CT_OUT_8_H * 0.10)  ) {
      valOutPWM_CARG = 0;  
  }

  //Serial.print(valOutPWM_CARG);
  
  
  analogWrite(pinDO_CARG, valOutPWM_CARG);

  // Entrada BMB_PERTURBACION
  valIn_PERT = analogRead(pinAI_PERT);
  
  // Se genera el PWM de la segunda entrada
  valOutPWM_PERT =  M_CONV_CT * ( valIn_PERT - CT_IN_10_L) ;

  if ( valOutPWM_PERT < (CT_OUT_8_H * 0.10) ) {
      valOutPWM_PERT = 0;
  }

  analogWrite(pinDO_PERT, valOutPWM_PERT);


  // Sensor ultrasónico
  imprimir_distancia();

  if ( digitalRead(btn_CALB) == LOW ){
    // La distancia ya fue calculada en imprimir_distancia()
    distancia_h = distancia;
  }

  // Se calcula nivel. La distancia ya fue calculada
  nivel = distancia_h - distancia;

  // Si es la primera vuelta del loop se establece el "nivel anterior" con el "nivel actual" sensado 
  if ( primer_vuelta ) {
    nivel_ant = nivel;
    primer_vuelta = false;  
  }
  
  nivel = a * nivel_ant  + (1 - a) * nivel;
  nivel_ant = nivel;

  analogWrite(pinDO_NVL, f_cm_ct8(nivel));

  // Impresion en pantalla OLED 
  imprimir_porcentaje_tanque();
  imprimir_tanque();
  imprimir_ciclos();
  

  u8g2.sendBuffer();          // transfer internal memory to the display

  delay(100);  
  
}

// Impresión de distancia
void imprimir_distancia() {
  char char_distancia[4];
  int base = 10;
  
  distancia = calcular_distancia();
  
  //String str_distancia = itoa( distancia ,char_distancia,base);
  //String str_distancia = ftoa( distancia ,char_distancia,base);
  //str_distancia = str_distancia + "cm";
  //u8g2.drawStr(70,30,str_distancia.c_str());

  //Se imprime distancia a tacho vacio
  String str_distancia = itoa( round(nivel) ,char_distancia,base);
  u8g2.drawStr(70,30,str_distancia.c_str());
  
}

// Calculo de distancia
double calcular_distancia(){
  double aux_dist = 0;
  long duracion_pulso = 0;
  
  digitalWrite(pinDO_TRIG, HIGH);       // generacion del pulso a enviar
  delayMicroseconds(10);                 // al pin conectado al trigger
  digitalWrite(pinDO_TRIG, LOW);        // del sensor
  
  duracion_pulso = pulseIn(pinDI_ECO, HIGH);  // con funcion pulseIn se espera un pulso alto en Echo
  aux_dist = duracion_pulso / 58.2;           // distancia medida en centimetros

  if (aux_dist >= distancia_h) {
    aux_dist = distancia_h;
  }

  delay(50);
  return aux_dist;
}

void imprimir_porcentaje_tanque() {
  // el valor entero del porcentaje
  int int_porcentaje = f_cm_perc(distancia);
  // para el valor de char
  char char_porcentaje[4];
  int base = 10;

  String str_porcentaje = itoa(int_porcentaje,char_porcentaje,base);
  str_porcentaje = str_porcentaje + "%";

  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  //u8g2.drawStr(35,30,itoa(int_porcentaje,str_porcentaje,base));
  //u8g2.setFont(u8g2_font_unifont_t_symbols);
  //u8g2.drawUTF8(46,30,"%");
  u8g2.drawUTF8(35,30,str_porcentaje.c_str());
}

void imprimir_ciclos() {
  // para el valor de char
  char char_porcentaje[4];
  int base = 10;

  String str_porcentaje = itoa(valOutPWM_CARG,char_porcentaje,base);
  str_porcentaje = str_porcentaje + " ct PWM";

  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawUTF8(60,50,str_porcentaje.c_str());
}

void imprimir_tanque() {
  aux_nivel = f_perc_px(f_cm_perc(distancia));
  u8g2.drawFrame(x_tacho,y_tacho,ancho_tacho,alto_tacho);
  while(aux_nivel < nivel_tacho_vacio) {
    u8g2.drawHLine(x_tacho,aux_nivel,ancho_tacho);
    aux_nivel = aux_nivel + 1;
  }
}
