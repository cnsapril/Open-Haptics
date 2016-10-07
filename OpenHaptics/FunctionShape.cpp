double funtemp1, funtemp2, funtemp3;
funtemp1 = radius - sqrt(x*x+z*z);
funtemp2 = height/2 - y;
funtemp3 = y + height/2;
funtemp = interFunction(funtemp1, funtemp2);
funtemp = interFunction(funtemp, funtemp3);
 
double interFunction(double xx, double yy)
{
       double fun = xx + yy - sqrt(xx*xx + yy*yy);// + 1/(1+xx*xx*10+yy*yy*10);
       return fun;
}
 
double unionFunction(double xx, double yy)
{
       //1. the parameters before xx and yy must be the same, otherwise the force is different when rendering drawing and tangible file.
       //2. this blending algorithm would make tangible models bigger, not match
       double fun = xx + yy + sqrt(xx*xx + yy*yy);// + 1/(1+xx*xx*25+yy*yy*25);
       return fun;
 
}
 
/*******************************************************************************
Updates the end effector position, and sets the new object position,
 calculates force.
*******************************************************************************/
void ContactModel::UpdateEffectorPosition(const hduVector3Dd visitor)
{
       double delta = 0.00001;
    double func = shapefunction(visitor[0], visitor[1], visitor[2]);
      
       m_effectorPosition = visitor;
       hduVector3Dd gradienttemp;//added
 
       //if((func>=0.0)&&(func<=15.0))
       if(func>=0.0)
       {
              isCollided = TRUE;
              /**added**/
              gradienttemp[0] = (shapefunction((visitor[0]+delta), visitor[1], visitor[2])-func)/delta;
              gradienttemp[1] = (shapefunction(visitor[0], (visitor[1]+delta), visitor[2])-func)/delta;
              gradienttemp[2] = (shapefunction(visitor[0], visitor[1], (visitor[2]+delta))-func)/delta;
             
              //Normalize gradient
              hduVecNormalizeInPlace(gradienttemp);
 
              hduVector3Dd temp1 = m_effectorPosition - m_contactPosition;
              HDdouble aa = temp1.dotProduct(gradienttemp);
              hduVector3Dd temp2 = m_effectorPosition - m_visitorPosition;
              HDdouble bb = temp2.dotProduct(gradienttemp);
 
 
       if((shapefunction(m_contactPosition[0]+gradienttemp[0],m_contactPosition[1]+gradienttemp[1],m_contactPosition[2]+gradienttemp[2])>0)
                     ||(aa>0)||(bb>0))
              {
                     gradienttemp[0] = -gradienttemp[0];
                     gradienttemp[1] = -gradienttemp[1];
                     gradienttemp[2] = -gradienttemp[2];
              }
              HDdouble cc = gradienttemp.dotProduct(gradient);
              HDdouble dd = gradienttemp.magnitude()*gradient.magnitude();
              dd = cc/dd;
             
 
              if(dd<0.8)
              {
                     gradient = gradient + gradienttemp;
                    
                     //cout<<"omg~~~"<<endl;
                     //cout<<"gradient: "<<gradient<<endl;
                     //cout<<"gradienttemp: "<<gradienttemp<<endl;
 
              }
              else
                     gradient = gradienttemp;
             
              hduVecNormalizeInPlace(gradient);
              /**added end**/
             
 
              m_visitorPosition = m_effectorPosition;
 
              /////////////////////////////////////////////////////////////////////////////////////////////
              //subdivision method, find the contact point on the surface
              /* calculate the distance from the hip to the last point outside the shape. */
              HDdouble distance = temp2.magnitude();
              hduVector3Dd increment = gradient*distance;
 
              //m_end is the first point out of object in the direction of gradient
              hduVector3Dd m_end = m_visitorPosition + increment;
              hduVector3Dd m_new;
              //delete the else part on 29/04
              if(shapefunction(m_end[0], m_end[1], m_end[2])<0)
              {
                     m_new = m_end;
                    
                     for(int i = 0; i <10; i++)
                     {
                           increment = m_end - m_visitorPosition;
                           m_new = m_visitorPosition + increment/2;
                           if(shapefunction(m_new[0], m_new[1], m_new[2])<0)
                                  m_end = m_new;
                           else
                                  m_visitorPosition = m_new;
                     }
 
              }
              else
              {
                     m_end = m_visitorPosition + increment;
                    
                     m_new = m_end;
                    
              }
      
             
              m_visitorPosition = m_new;
 
 
              /////////////////////////////////////////////////////////////////////////////////////
 
 
              increment = m_visitorPosition - m_effectorPosition;
              distance = increment.magnitude();
              HDdouble k = 1.0;
              m_forceOnVisitor = k*distance*gradient;
              //cout<<gradient<<"   "<<endl;
              //cout<<m_forceOnVisitor.magnitude()<<endl;
 
#if 0
              if(proxy_count%20 == 0)
              {
                     m_proxy[proxy_count/20].proxy_pos = m_effectorPosition;
                     m_proxy[proxy_count/20].impnormal = gradient;
              }
              proxy_count++;
#endif
 
              /////////////////////////////////////////////////////////////////////////////////////////////
 
 
       }
       else
       {
              proxy_count = proxy_count - proxy_count%20;
              isCollided = FALSE;
              //when the force is 0, the contact point and the hip position are the same.
              m_contactPosition = m_effectorPosition;
 
              m_forceOnVisitor.set(0.0, 0.0, 0.0);
              m_visitorPosition = m_effectorPosition;
              gradient[0] = 0;
              gradient[1] = 0;
              gradient[2] = 0;
              //if(state == true)
              //     state=false;
              proxyPosition = m_contactPosition;
       }
 
}