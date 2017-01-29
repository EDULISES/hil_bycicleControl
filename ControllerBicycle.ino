 /*
 *------------------------------------------------------------------------
 * Product: Controller for mathemathical model of a bicycle.
 *
 * Filename: ControllerBicycle.ino
 *
 * Author: Edward Benitez
 *
 * File Description: This is the implementation of controller for 
 * mathemathical model of a bicycle.
 *
 *
 *------------------------------------------------------------------------
 * File History
 *
 * Date   Init  Notes
 *------------------------------------------------------------------------
 * 27/01/17 ebenitez  Created.
 *------------------------------------------------------------------------
 */

#define N  (2)

#include <Utility.h>
#include <MatrixMath.h>

char strData[256];
typedef unsigned char uint8;
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Control output
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
float Mz = 0.0;
float Dc = 0.0;
float dc = 0.0;
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
const float Lf = 1.17; 
const float Lr = 1.43; 
const float Cf = 1.81;
const float Cr = 1.68;
const float Bf = 7.2;
const float Br = 11.0;
const float Dr = 8394.0;
const float Df = 8854.0;
const float m = 1480.0; 
const float Jz = 2300.0;

const float mu = 0.9;
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Control gain
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
const float M1=10.0;
const float M2=3.5;
const float M3=1.0;
const float M4=1.0;
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Variables for integration
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
float v1Ant = 0.0; 
float v2Ant = 0.0; 
float tm = 1.0;
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/

void setup() {
  Serial.begin(250000);
  Serial2.begin(250000);
  delay(100);
  startSimulation();
}

void loop() {
  if(Serial2.available() > 0){
    char data = Serial2.read();
    if(data == '|'){   
      unsigned char index = 0;
      strData[index] = data;
      while(data != '\n'){
        if(Serial2.available() > 0){
          data = Serial2.read();
          index ++;
          strData[index] = data;  
        }
      }
      //Serial.print(strData);
      extractVar(strData);
      sendOutSystem();
    }
  }      
}

signed int findSymbol(const char *_str, char _symbol){
  int i = 0;
  int _lengthStr = strlen(_str);
  while(i <= _lengthStr){
    if(_str[i] == _symbol){
      return i;
      break;
    }
    i++;
  }
  return -1;
}

void subStr(const char *_str, char*_subStr, uint8 _startIndex, uint8 _endIndex){
  int k = 0;
  for(k = _startIndex; k <= _endIndex; k++) {
    if(k != _endIndex) {
      _subStr[k - _startIndex] = _str[k];
    }
    else {
      _subStr[k - _startIndex] = '\0';
    }
  }
}

void extractVar(char *_str){  
  char vxStr[24], vyStr[24], wzStr[24], vyRefStr[24], wzRefStr[24], ddStr[24];
  uint8 _startStr = findSymbol(_str, '|');
  uint8 _endStr = findSymbol(_str, ',');
  uint8 _endStrData = findSymbol(_str, '\n');
  subStr(_str, vxStr, _startStr + 1, _endStr);

  uint8 _lenghtD1 = _endStrData - (_endStr + 1);
  char _d1[_lenghtD1 + 1];
  _startStr = _endStr + 1;
  subStr(_str, _d1, _startStr, _endStrData);
  _endStr = findSymbol(_d1, ',');
  subStr(_d1, vyStr, 0, _endStr);

  uint8 _lenghtD2 = _lenghtD1 - (_endStr + 1);
  char _d2[_lenghtD2 + 1];
  _startStr = _endStr + 1;
  subStr(_d1, _d2, _startStr, _endStrData);
  _endStr = findSymbol(_d2, ',');
  subStr(_d2, wzStr, 0, _endStr);

  uint8 _lenghtD3 = _lenghtD2 - (_endStr + 1);
  char _d3[_lenghtD3 + 1];
  _startStr = _endStr + 1;
  subStr(_d2, _d3, _startStr, _endStrData);
  _endStr = findSymbol(_d3, ',');
  subStr(_d3, vyRefStr, 0, _endStr);

  uint8 _lenghtD4 = _lenghtD3 - (_endStr + 1);
  char _d4[_lenghtD4 + 1];
  _startStr = _endStr + 1;
  subStr(_d3, _d4, _startStr, _endStrData);
  _endStr = findSymbol(_d4, ',');
  subStr(_d4, wzRefStr, 0, _endStr);

  uint8 _lenghtD5 = _lenghtD4 - (_endStr + 1);
  char _d5[_lenghtD5 + 1];
  _startStr = _endStr + 1;
  subStr(_d4, _d5, _startStr, _endStrData);
  strcpy(ddStr, _d5);
  
  float vx = atof(vxStr);  
  float vy = atof(vyStr);
  float wz = atof(wzStr);
  float vyref = atof(vyRefStr);
  float wzref = atof(wzRefStr);
  float dd = atof(ddStr);  
  
  control(vx, vy, wz, vyref, wzref, dd);
  Serial.print(vxStr);
  Serial.print(",");
  Serial.print(vx);
  Serial.print(",");
  Serial.print(vyStr);
  Serial.print(",");
  Serial.print(vy,4);
  Serial.print(",");
  Serial.print(wzStr);
  Serial.print(",");
  Serial.print(wz,4);
  Serial.print(",");
  Serial.print(vyRefStr);
  Serial.print(",");
  Serial.print(vyref,4);
  Serial.print(",");
  Serial.print(wzRefStr);
  Serial.print(",");
  Serial.print(wzref,4);  
  Serial.print(",");
  Serial.print(ddStr);
  Serial.print(",");
  Serial.println(dd,4);
  /*Serial.print("\t");
  Serial.print(tActStr);
  Serial.print("\t");
  Serial.print(tAct,4);*/
  
}

void control(float _vx, float _vy, float _wz, float _vyRef, float _wzRef, float _dd){
  float alphar = -(_vy - Lr * _wz)/_vx;
  float alphaf0 = _dd - (_vy + Lf * _wz)/_vx;
  /*Serial.print(alphar,4);
  Serial.print("\t");
  Serial.println(alphaf0,4);*/
  
  float phir = sin(Cr * atan(Br * alphar));
  float phif0 = sin(Cf * atan(Bf * alphaf0));
  /*Serial.print(phir,4);
  Serial.print("\t");
  Serial.println(phif0,4);*/

  float B[2][2];
  B[0][0] = mu*Df/m;
  B[0][1] = 0;
  B[1][0] = mu*Df*Lf/Jz;
  B[1][1] = 1/Jz;

  /*Serial.print(B[0][0],4);
  Serial.print("\t");
  Serial.println(B[0][1],4);
  Serial.print("\t");
  Serial.print(B[1][0],4);
  Serial.print("\t");
  Serial.println(B[1][1],4);*/

  float eVy = _vy - _vyRef;
  float eWz = _wz - _wzRef;
  /*Serial.print(eVy,4);
  Serial.print("\t");
  Serial.println(eWz,4);*/
  
  float dv1 = -M3 * sign(eVy);
  float dv2 = -M4 * sign(eWz);
  float v1Act = v1Ant + tm*dv1;
  float v2Act = v2Ant + tm*dv2;
  /*Serial.print(dv1,4);
  Serial.print("\t");
  Serial.println(dv2,4);*/  
  
  
  float C[2][1];
  float D[2][2];
  float u[1][2];
  C[0][0] = -M1 * sqrt(abs(eVy)) * sign(eVy) + v1Act;
  C[1][0] = -M2*sqrt(abs(eWz))* sign(eWz) + v2Act;
  v1Ant = v1Act;
  v2Ant = v2Act;
  /*Serial.print(C[0][0],4);
  Serial.print("\t");
  Serial.println(C[1][0],4);*/
  
  //[-M1 * sqrt(abs(eVy)) * sign(eVy) + v(1); -M2*sqrt(abs(eWz))*sign(eWz) + v(2)];
  Matrix.Invert((float*)B,N);
  /*Serial.print(B[0][0],4);
  Serial.print("\t");
  Serial.println(B[0][1],4);*/
  /*Serial.print("\t");
  Serial.print(B[1][0],4);
  Serial.print("\t");
  Serial.println(B[1][1],4);*/
  
  Matrix.Multiply((float*)B,(float*)C,N,N,N,(float*)u); 
  Mz = u[1][2];
  Dc = u[1][1];
    
  if(Dc > (1-phif0)){
    Dc = (1-phif0);
  }
  if(Dc < (-1 - phif0)){
    Dc = -1 - phif0;
    //Mz1=-(Df*mu*phif0*Lf-Dr*mu*phir*Lr)-mu*Df*Lf*Dcreal+Jz*dxref(2);
  }

  float phif = Dc + phif0;
  dc = -_dd + (_vy + Lf * _wz) / _vx + (1/Bf)*tan((1/Cf)*asin((phif)));
}

void sendOutSystem(){
  //Max presicion with Arduino: 9 decimals. Rounding the final decimal number
  Serial2.print(Mz,9);
  //Serial.print(Mz,9);
  Serial2.print(",");  
  //Serial.print(",");
  Serial2.println(dc,9); 
  //Serial.println(dc,9);
}

void startSimulation(){
  char _strCmd[10];  
  while(true){
    if(Serial2.available() > 0){
      char _startCmd = Serial2.read();
      if(_startCmd == '|'){   
        unsigned char _indx = 0;
        _strCmd[_indx] = _startCmd;
        while(_startCmd != '\n'){
          if(Serial2.available() > 0){
            _startCmd = Serial2.read();
            _indx ++;
            _strCmd[_indx] = _startCmd;  
          }          
        }        
        uint8 _startStr = findSymbol(_strCmd, '|');
        uint8 _endStr = findSymbol(_strCmd, '\n');
        char auxStr[_endStr];       
        subStr(_strCmd, auxStr, _startStr, _endStr);        
        if(strcmp(auxStr, "|start") == 0){
          Serial.print(_strCmd);
          Serial2.flush();          
          break;
        }        
      }
    }    
  }
}

