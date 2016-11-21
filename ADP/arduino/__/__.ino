float X[3]={0};
float Y[3]={0};
float H[3]={-110};
float CX[11]={0};
float CY[11]={0};
float CH[11]={-110};
float sx,sy,sh;
float zx[5],zy[5],zh[5];
String comdata;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  serialdeal();
}
float getpoint(float *shuzu)
{
  float re=shuzu[0];
  for(int i=0;i<11;i++){
    shuzu[i]=shuzu[i+1];
  }
  return re;
}
//计算新增的五个点 并加入数组
void computedvalue(float secpoint,float *z,float *dvalue)
{
  float firpoint=z[2];
  for(int i=0;i<2;i++){
    z[i]=z[i+1];
  }
  z[2]=secpoint;
  for(int i=0;i<4;i++){
    dvalue[i]=firpoint+(secpoint-firpoint)*(i+1)/5;
  }
  dvalue[4]=secpoint;
}
//把新增的五个点 加进细分数组
void addtoc(float *dvalue,float *csz)
{
  for(int i=0;i<5;i++){
    csz[i+6]=dvalue[i];
  }
}  
void serialdeal(){
     while (Serial.available() > 0 )  
    {
      char com=char(Serial.read());
      if(com==' '){
        break;
      }
      comdata += com;
      delay(6); 
    }
    if (comdata.length() > 0)
    {
      int yindex=0;
      int hindex=0;
      
      Serial.println(comdata);
      switch(comdata[0]){
        case 'x':
  
            //isdrip=true;
            yindex=comdata.indexOf('y',0);
            hindex=comdata.indexOf('h',0);
            sx=comdata.substring(1,yindex).toInt();
            //if comdata contain the 'h' get the h 
            if(hindex>0){
              sh=comdata.substring(hindex+1).toInt();//h 坐标偏移100 客户端初始50 即-50
              sy=comdata.substring(yindex+1,hindex).toInt();
            }else {
              sy=comdata.substring(yindex+1).toInt();
            }
            computedvalue(sx,X,zx);
            computedvalue(sy,Y,zy);
            computedvalue(sh-125,H,zh);
            addtoc(zx,CX);
            addtoc(zy,CY);
            addtoc(zh,CH);
            for(int i=0;i<5;i++){
              sx=getpoint(CX);
              sy=getpoint(CY);
              sh=getpoint(CH);
              Serial.println(String("x")+sx+"y"+sy+"h"+sh);
            }
        break;//change the "x"and"y" it is possible that the high change as well;
      }
      comdata = "";
    }
}
