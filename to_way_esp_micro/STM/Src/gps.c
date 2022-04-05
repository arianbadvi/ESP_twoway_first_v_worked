#include "gps.h"

extern char gps_validation_test;

void  gps_data_check_extrac_func (char *Str, char *Utc_time, char* Validation,
                                  char *Latitude, char *Ns, char *Longitude, 
                                  char *Ew)
{ 
  
  struct INDEX
  {
    unsigned char       Utc;
    unsigned int        data;
    unsigned char       latitude;
    unsigned char       longitude;
  };
  
  enum Gps_ex_state       //main menu
  {
    waiting,
    start_of_datapack,
    gps_validation_state,
    data_extraction,
    data_error,
  };
  enum Gpmode
  {
    S,
    G,
    P,
    R,
    NR,   //NOT R, RMS
    ER,   //ERROR
  };
  enum Comma_count
  {
    zero,
    one_utc,
    two_validation,
    three_latitude,
    four_ns,
    five_longitude,
    six_ew,
    seven,
  };  
  
  volatile unsigned char Progress = ACTIVE;
  volatile static char Check_ch = 0;
  volatile static enum Gps_ex_state    Main_state = waiting;
  volatile static enum Gpmode  Gp_state = S;
  volatile static enum Comma_count      Comma_state;
  static struct INDEX  Index, Index_default = {0, 0, 0, 0};
  
  Main_state = waiting;
  Gp_state = S;
  Comma_state = zero;
  Index = Index_default;
  

  *Validation = 0; 
  /*
  this is for checking the connection of GPS
  the validation will update and check in each data extraction
  */
  
  while (Progress == ACTIVE)
  {
    if ( (*(Str + Index.data)) != 0 )   Check_ch = *(Str + Index.data);
    else
    {
      //put_str("it was zero\n", &huart2);
      Progress = INACTIVE;
      break;
    }
        
    //Data_index++;
    Index.data++;
    
    switch (Main_state)
    {
      case waiting:
        Main_state = (Check_ch == 36) ? start_of_datapack: waiting;   //36 => $     
      break;
      
      case start_of_datapack:
        switch(Gp_state)
        {
          case S:
            //put_str("THIS WAS S\n", &huart2);
            Gp_state = (Check_ch == 'G') ? G: ER;
          break;          
          
          case G:
            //put_str("THIS WAS G\n", &huart2);
            Gp_state = (Check_ch == 'P') ? P: ER;
          break;          
          
          case P:
            //put_str("THIS WAS P\n", &huart2);
            Gp_state = (Check_ch == 'R') ? R: NR;
          break;        
          
          case R:  
            //put_str("THIS WAS R\n", &huart2);
            Main_state = gps_validation_state;
          break;        
         
          case NR:
            //put_str("THIS WAS NR\n", &huart2);
            Main_state = waiting;
            Gp_state = S;
            //Progress = INACTIVE;
          break;        
          
          case ER:
            //put_str("THIS WAS ER\n", &huart2);
            Gp_state = S;
            Main_state = waiting;
            Progress = INACTIVE;
          break;       
        }
        
      break;
      
      case gps_validation_state:
        switch(Comma_state)
        {
        case zero:
          Comma_state = (Check_ch == 44) ? one_utc: zero;          //44 => ,
        break;
        
        case one_utc:
          Comma_state = (Check_ch == 44) ? two_validation: one_utc;
          *(Utc_time + Index.Utc) = (Comma_state == one_utc) ? Check_ch : '\t';
          Index.Utc ++;
          
        break;
        
        case two_validation:
          //sprintf(Utc_time, "%s\n", Utc_time);
          *(Utc_time + Index.Utc) = 0;         
          //put_str(Utc_time, &huart2);
          
          //test
          gps_validation_test = Check_ch;
          switch (Check_ch)
          {
            case 'V':
              Main_state = data_error;
              *Validation = 'V';
            break;
            
            case 'A':
              Main_state = data_extraction;
              *Validation = 'A';
            break; 
          }
          
        break;
        }   
        
      break; 
     
      case data_extraction:
        //put_str("THIS WAS data extracton\n", &huart2);        
        switch(Comma_state)
        {
        case two_validation:
          Comma_state = (Check_ch == 44) ? three_latitude : two_validation; 

        break;
        
        case three_latitude:
          Comma_state = (Check_ch == 44) ? four_ns: three_latitude;          //44 => ,
          *(Latitude + Index.latitude) = (Comma_state == three_latitude) ? Check_ch : '\t';
          Index.latitude ++;
          
        break;
        
        case four_ns:
          *(Latitude + Index.latitude) = 0;
          Comma_state = (Check_ch == 44) ? five_longitude: four_ns;
          switch (Check_ch)
          {
            case 'N':
              *Ns = 'N';            
            break;
            
            case 'S':
              *Ns = 'S';            
            break;            
          }
          
        break;
        
        case five_longitude:
          Comma_state = (Check_ch == 44) ? six_ew: five_longitude;
          *(Longitude + Index.longitude) = (Comma_state == five_longitude) ? Check_ch : '\t';
          Index.longitude ++;
        
        break;
        
        case six_ew:
          
          *(Longitude + Index.longitude) = 0;
          Comma_state = (Check_ch == 44) ? seven: six_ew;
          switch (Check_ch)
          {
            case 'E':
              *Ew = 'E';            
            break;
            
            case 'W':
              *Ew = 'W';            
            break;            
          }
          
        break;
        
        case seven:
            Progress = INACTIVE;
        break;
                
        }
         
      break;
    
      case data_error:
        //put_str("THIS WAS dataError\r\n", &huart2);
        Progress = INACTIVE;
         
      break;    
    }   //switch
  }     //while       
}

void gps_conv_double (char *In, float *Out)
{
  float Float_buffer = 0;
  sscanf(In, "%f", &Float_buffer);
  *Out = Float_buffer;
}

void gps_cor_final_conv_func(float In, float *Out)
{
  int Decimal_part = 0;
  float Min_part = 0.0;
  Decimal_part = (int)(In / 100);
  Min_part = In - (Decimal_part * 100) ;
  Min_part /= 60;
  *Out = Decimal_part + Min_part;
}

void gps_utc_conv_func (char *Utc_time, unsigned char *Hour, unsigned char *Min,
                        unsigned char *Sec)
{
  unsigned int time_buffer = 0;
  unsigned char Hour_buffer = 0, Min_buffer = 0;
  sscanf(Utc_time, "%d", &time_buffer);
  Hour_buffer = (unsigned char)(time_buffer / 10000);
  Min_buffer = (unsigned char) ((time_buffer - (Hour_buffer * 10000)) / 100);
  *Sec = (unsigned char) (time_buffer - ((Hour_buffer * 10000) + (Min_buffer * 100)));

  Min_buffer += 30;
  if (Min_buffer > 59)
  {
    Min_buffer -= 60;
    Hour_buffer += 1;
  }

  Hour_buffer += 3;
  if (Hour_buffer > 23) Hour_buffer -= 24;
  
  *Hour = Hour_buffer;
  *Min = Min_buffer;
}