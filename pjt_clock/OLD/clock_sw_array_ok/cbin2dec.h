char adones;
char adtenths;
char adhundredths;
char adthousandths;

char tenthous ;
char thousands ;					
char hundreds ;
char tens ;
char ones ;

void adc2dec( unsigned int ADRES )
{
	 adones = 0;					//reset values
	 adtenths = 0;
	 adhundredths = 0;
	 adthousandths = 0;
	 
	 while ( ADRES > 0x0 )
	 {
	  	if( ADRES > 0x333 )		//test for 1 volt or greater
		{
			adones++;			//increment 1 volt counter
			ADRES -= 0x334;			//subtract 1 volt
		}
		else if( ADRES > 0x51 && ADRES <= 0x333 )		
		{
			if (adtenths < 9)
		   	{
			  adtenths++;			//increment tenths
			}
			else 
			{
			  adones++;			//tenths has rolled over
			  adtenths = 0;			//so increment ones and reset tenths
			}
			ADRES -=0x52;
		}
		else if(ADRES > 0x8 && ADRES <= 0x51)		
		{
			if (adhundredths < 9)
			{
				adhundredths++;		//increment hundreths
			}
			else 
			{
				
				adhundredths = 0;	//reset hundredths
				if (adtenths < 9)
		   		{
			  		adtenths++;			//and increment tenths
				}
				else 
				{
			 	 	adones++;			//unless tenths has rolled over
			  		adtenths = 0;			//so increment ones and reset tenths
				}
			}
			ADRES -= 0x9;
		}
		else if(ADRES >= 0x1 && ADRES <= 0x8)		
		{
			if (adthousandths < 9)
			{
				adthousandths++;		//increment thousandths
			}
			else
			{					//unless thousands has rolled over
				adthousandths = 0;		//so reset thousands 
				if (adhundredths < 9)		
				{
					adhundredths++;		//and increment hundreths
				}
				else 
				{				//unless hundredths has rolled over
					adhundredths = 0;	//so reset hundredths
					if (adtenths < 9)
		   			{
			  			adtenths++;			//and increment tenths
					}
					else 
					{
			 	 		adones++;			//unless tenths has rolled over
			  			adtenths = 0;			//so increment ones and reset tenths
					}
				}
			}
			ADRES -= 1;
		}
		
	 }
	adones += 0x30;			//convert all values to ascii
	adtenths += 0x30;
	adhundredths += 0x30;
	adthousandths += 0x30;
	
} 


void bin2dec( unsigned long int ADRES )
{
	 tenthous = 0;
	 thousands = 0;					//reset values
	 hundreds = 0;
	 tens = 0;
	 ones = 0;
	 while (ADRES > 0x0)
	 {
	  	
	  	if (ADRES > 0x270F)
	  	{
	  		tenthous++;
	  		ADRES -= 0x2710;
	  	}
	  	else if(ADRES > 0x03E7 && ADRES <= 0x270F)		
		{
			if (thousands < 9)
			{
				thousands++;			
			}
			else
			{
				tenthous++;
				thousands=0;
			}
			ADRES -= 0x3E8;			
		}
		else if(ADRES > 0x63 && ADRES <= 0x3E7)		
		{
			if (hundreds < 9)
		   	{
			  	hundreds++;			
			}
			else 
			{
			  	hundreds = 0;			
			  	if (thousands < 9)
				{
					thousands++;			
				}
				else
				{
					tenthous++;
					thousands=0;
				}
			}
			ADRES -=0x64;
		}
		else if(ADRES > 0x9 && ADRES <= 0x63)		
		{
			if (tens < 9)
			{
				tens++;		
			}
			else 
			{
				
				tens = 0;	
				if (hundreds < 9)
		   		{
			  		hundreds++;			
				}
				else 
				{
			  		hundreds = 0;			
			  		if (thousands < 9)
					{
						thousands++;			
					}
					else
					{
						tenthous++;
						thousands=0;
					}	
				}
			}
			ADRES -= 0xA;
		}
		else if(ADRES >= 0x1 && ADRES <= 0x9)		
		{
			if (ones < 9)
			{
				ones++;		
			}
			else
			{
				ones = 0;
				if (tens < 9)
				{
					tens++;		
				}
				else 
				{
					tens = 0;
					if (hundreds < 9)
		   			{
			  			hundreds++;
					}
					else 
					{
			  			hundreds = 0;
			  			if (thousands < 9)
						{
							thousands++;
						}
						else
						{
							tenthous++;
							thousands=0;
						}
					}
				}
			}
			ADRES -= 1;
		}
		
	 }
	tenthous += 0x30;		//Store all conversions in ASCII form
	thousands += 0x30;
	hundreds += 0x30;
	tens += 0x30;
	ones += 0x30;
	
} 
