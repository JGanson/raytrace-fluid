#include <fstream>
#include <iomanip>
#include <algorithm>
#include <iostream>

#include "fluid.h"
#include "../argparser.h"
#include "../boundingbox.h"
#include "marching_cubes.h"
#include "../utils.h"
#include "../meshdata.h"



void Fluid::calculate_surface(int i, int j, int k){
  double total = 0;
  int count = 0;
  // count the outgoing flow
  if(getCell(i+1,j,k)->getStatus()!=CELL_EMPTY){
    total += get_new_u_plus(i,j,k);
  }
  else{count++;}
  if(getCell(i-1,j,k)->getStatus()!=CELL_EMPTY){
    total -= get_new_u_plus(i-1, j, k);
  }
  else{count++;}
  if(getCell(i,j+1,k)->getStatus()!=CELL_EMPTY){
    total += get_new_v_plus(i,j,k);
  }
  else{
    count ++;
  }
  if(getCell(i,j-1,k)->getStatus()!=CELL_EMPTY){
    total -= get_new_v_plus(i,j-1,k);
  }
  else{count++;}
  

  // go in flow
  double avg = -1*total/count;

  if(getCell(i+1,j,k)->getStatus()==CELL_EMPTY){
    set_new_u_plus(i,j,k,-1*avg);
  }
  if(getCell(i-1,j,k)->getStatus()==CELL_EMPTY){
    set_new_u_plus(i,j,k,avg);
  }
 
  if(getCell(i,j+1,k)->getStatus()==CELL_EMPTY){
    set_new_v_plus(i,j,k,-1*avg);
  }
  if(getCell(i,j-1,k)->getStatus()==CELL_EMPTY){
    set_new_v_plus(i,j,k,avg);
  }
  
}



void Fluid::incompressibility_surface(int i, int j, int k){
  int count = 0;
  
  if(getCell(i,j,k)->getStatus() != CELL_SURFACE){
    return;
  }
  if(i-1 > -1 && getCell(i-1, j,k)->getStatus()!=CELL_EMPTY){
    count++;
  }
  if(i+1<nx && getCell(i+1,j,k)->getStatus()!=CELL_EMPTY){
    count++;
  }
  if(j-1>-1 && getCell(i,j-1,k)->getStatus()!=CELL_EMPTY){
    count++;
  }
  if(j+1<ny && getCell(i,j+1,k)->getStatus()!=CELL_EMPTY){
    count++;
  }
    
  if(count==3){
    
    if(getCell(i-1, j,k)->getStatus()==CELL_EMPTY){
      
      double value = get_new_u_plus(i,j,k) - (dx/dy)*(get_new_v_plus(i,j-1,k)-get_new_v_plus(i,j,k));
      getCell(i-1,j,k)->set_new_u_plus(value);
    }
    else if(getCell(i+1,j,k)->getStatus()==CELL_EMPTY){
      
      double value = get_new_u_plus(i-1,j,k) - (dx/dy)*(get_new_v_plus(i,j,k)-get_new_v_plus(i,j-1,k));
      getCell(i,j,k)->set_new_u_plus(value);
    }
    else if(getCell(i,j-1,k)->getStatus()==CELL_EMPTY){
      
      double value = get_new_v_plus(i,j,k) - (dy/dx)*(get_new_u_plus(i-1,j,k)-get_new_u_plus(i,j,k));
      getCell(i,j-1,k)->set_new_v_plus(value);
    }
    else if(getCell(i,j+1,k)->getStatus()==CELL_EMPTY){
      double value = get_new_v_plus(i,j-1,k) - (dy/dx)*(get_new_u_plus(i,j,k)-get_new_u_plus(i-1,j,k));
      getCell(i,j,k)->set_new_v_plus(value);
    }
    return;
    //calculate_surface()
  }

  if(count==2 || count == 1 ){
  
    if(getCell(i-1, j,k)->getStatus()==CELL_EMPTY && getCell(i+1,j,k)->getStatus()!=CELL_EMPTY ){
      //if(j!=ny-1 && i+1<nx){
      getCell(i-1, j,k)->set_new_u_plus(getCell(i,j,k)->get_new_u_plus());//}
    }
    if(getCell(i+1,j,k)->getStatus()==CELL_EMPTY&& getCell(i-1,j,k)->getStatus()!=CELL_EMPTY){
      //if( j!=ny-1&&i-1>-1){
      getCell(i, j,k)->set_new_u_plus(getCell(i-1,j,k)->get_new_u_plus());//}
    }
    //std::cout<<nx<<","<<ny<<std::endl;
    if(getCell(i,j-1,k)->getStatus()==CELL_EMPTY&& getCell(i,j+1,k)->getStatus()!=CELL_EMPTY){
      //if(j+1<ny){
        //std::cout<<i<<","<<j<<","<<k<<","<<count<<std::endl;
        getCell(i, j-1,k)->set_new_v_plus(getCell(i,j,k)->get_new_v_plus());
      //} 
    }
    if(getCell(i,j+1,k)->getStatus()==CELL_EMPTY&& getCell(i,j-1,k)->getStatus()!=CELL_EMPTY){
      //if(j-1>-1){
        getCell(i, j,k)->set_new_v_plus(getCell(i,j-1,k)->get_new_v_plus());
      //} 
    }
    /**/
    if(getCell(i,j+1,k)->getStatus()!=CELL_EMPTY&& getCell(i,j-1,k)->getStatus()!=CELL_EMPTY){
      if(j-1 > -1 && j+1<ny){
        //set_new_u_plus(i-1,j,k,get_new_v_plus(i,j,k));
        //set_new_u_plus(i,j,k,get_new_v_plus(i,j-1,k));
        double avg = get_new_v_plus(i,j,k) + get_new_v_plus(i,j-1,k);
        //std::cout<<"yes"<<std::endl;
        set_new_v_plus(i,j,k,avg/2);
        set_new_v_plus(i,j-1,k,avg/2);
      }
     
    }
    if(getCell(i+1,j,k)->getStatus()!=CELL_EMPTY&& getCell(i-1,j,k)->getStatus()!=CELL_EMPTY){
      if(i-1 > -1 && i+1 < nx){
        //set_new_v_plus(i,j-1,k,get_new_u_plus(i,j,k));
        //set_new_v_plus(i,j,k,get_new_u_plus(i-1,j,k));
        //set_new_u_plus(i,j,k,get_new_u_plus(i-1,j,k));
        double avg = get_new_u_plus(i,j,k) + get_new_u_plus(i-1,j,k);
        //std::cout<<"yes"<<std::endl;
        set_new_u_plus(i,j,k,avg/2);
        set_new_u_plus(i-1,j,k,avg/2);
      }
      
      
    }
  }
  //set_new_u_plus(i,j,k,get_new_u_plus(i,j,k)/2);
  //set_new_v_plus(i,j,k,get_new_v_plus(i,j,k)/2);
}


void Fluid::incompressibility_surface3D(int i, int j, int k){
  int count = 0;
  
  if(getCell(i,j,k)->getStatus() != CELL_SURFACE){
    return;
  }
  if(i-1 > -1 && getCell(i-1, j,k)->getStatus()!=CELL_EMPTY){
    count++;
  }
  if(i+1<nx && getCell(i+1,j,k)->getStatus()!=CELL_EMPTY){
    count++;
  }
  if(j-1>-1 && getCell(i,j-1,k)->getStatus()!=CELL_EMPTY){
    count++;
  }
  if(j+1<ny && getCell(i,j+1,k)->getStatus()!=CELL_EMPTY){
    count++;
  }
  if(k-1>-1 && getCell(i,j,k-1)->getStatus()!=CELL_EMPTY){
    count++;
  }
  if(k+1<nz && getCell(i,j,k+1)->getStatus()!=CELL_EMPTY){
    count++;
  }
  

  // when there is only one open side
  if(count==5){    
    if(getCell(i-1, j,k)->getStatus()==CELL_EMPTY){
      double value = get_new_u_plus(i,j,k) - (dx/dy)*(get_new_v_plus(i,j-1,k)-get_new_v_plus(i,j,k))
      -(dx/dz)*(get_new_w_plus(i,j,k-1)-get_new_w_plus(i,j,k));
      getCell(i-1,j,k)->set_new_u_plus(value);
    }
    else if(getCell(i+1,j,k)->getStatus()==CELL_EMPTY){
      double value = get_new_u_plus(i-1,j,k) - (dx/dy)*(get_new_v_plus(i,j,k)-get_new_v_plus(i,j-1,k))
      -(dx/dz)*(get_new_w_plus(i,j,k)-get_new_w_plus(i,j,k-1));
      getCell(i,j,k)->set_new_u_plus(value);
    }
    else if(getCell(i,j-1,k)->getStatus()==CELL_EMPTY){
      
      double value = get_new_v_plus(i,j,k) - (dy/dx)*(get_new_u_plus(i-1,j,k)-get_new_u_plus(i,j,k))
      - (dy/dz)*(get_new_w_plus(i,j,k-1)-get_new_w_plus(i,j,k));
      getCell(i,j-1,k)->set_new_v_plus(value);
    }
    else if(getCell(i,j+1,k)->getStatus()==CELL_EMPTY){
      double value = get_new_v_plus(i,j-1,k) - (dy/dx)*(get_new_u_plus(i,j,k)-get_new_u_plus(i-1,j,k))
      -(dy/dz)*(get_new_w_plus(i,j,k)-get_new_w_plus(i,j,k-1));
      getCell(i,j,k)->set_new_v_plus(value);
    }
    else if(getCell(i,j,k-1)->getStatus()==CELL_EMPTY){
      
      double value = get_new_w_plus(i,j,k) - (dz/dx)*(get_new_u_plus(i-1,j,k)-get_new_u_plus(i,j,k))
      - (dz/dy)*(get_new_v_plus(i,j-1,k)-get_new_v_plus(i,j,k));
      getCell(i,j-1,k)->set_new_w_plus(value);
    }
    else if(getCell(i,j,k+1)->getStatus()==CELL_EMPTY){
      double value = get_new_w_plus(i,j,k-1) - (dz/dx)*(get_new_u_plus(i,j,k)-get_new_u_plus(i-1,j,k))
      -(dz/dy)*(get_new_v_plus(i,j,k)-get_new_v_plus(i,j-1,k));
      getCell(i,j,k)->set_new_w_plus(value);
    }
  }
  
  else{
    if(getCell(i-1, j,k)->getStatus()==CELL_EMPTY && getCell(i+1,j,k)->getStatus()!=CELL_EMPTY ){
      if(j!=ny-1 ){
      getCell(i-1, j,k)->set_new_u_plus(getCell(i,j,k)->get_new_u_plus());
      }
    }
    if(getCell(i+1,j,k)->getStatus()==CELL_EMPTY&& getCell(i-1,j,k)->getStatus()!=CELL_EMPTY){
      if( j!=ny-1){
      getCell(i, j,k)->set_new_u_plus(getCell(i-1,j,k)->get_new_u_plus());
      }
    }
    if(getCell(i,j-1,k)->getStatus()==CELL_EMPTY&& getCell(i,j+1,k)->getStatus()!=CELL_EMPTY){
        getCell(i, j-1,k)->set_new_v_plus(getCell(i,j,k)->get_new_v_plus());
    }
    if(getCell(i,j+1,k)->getStatus()==CELL_EMPTY&& getCell(i,j-1,k)->getStatus()!=CELL_EMPTY){
        getCell(i, j,k)->set_new_v_plus(getCell(i,j-1,k)->get_new_v_plus());
    }
    if(getCell(i,j,k-1)->getStatus()==CELL_EMPTY&& getCell(i,j,k+1)->getStatus()!=CELL_EMPTY){
        getCell(i, j,k-1)->set_new_w_plus(getCell(i,j,k)->get_new_w_plus());
    }
    if(getCell(i,j,k+1)->getStatus()==CELL_EMPTY&& getCell(i,j,k-1)->getStatus()!=CELL_EMPTY){
        getCell(i, j,k)->set_new_w_plus(getCell(i,j,k-1)->get_new_w_plus());
    }
    

    if(getCell(i-1, j,k)->getStatus()!=CELL_EMPTY && getCell(i+1,j,k)->getStatus()!=CELL_EMPTY ){
      if(i-1 > -1 && i+1 < nx){
        //getCell(i, j,k)->set_new_u_plus(getCell(i-1,j,k)->get_new_u_plus());

        double avg = get_new_u_plus(i,j,k) + get_new_u_plus(i-1,j,k);
        set_new_u_plus(i,j,k,avg/2);
        set_new_u_plus(i-1,j,k,avg/2);
      }
    }
    if(getCell(i,j+1,k)->getStatus()!=CELL_EMPTY&& getCell(i,j-1,k)->getStatus()!=CELL_EMPTY){
      if(j-1 > -1 && j+1<ny){
        //getCell(i, j,k)->set_new_v_plus(getCell(i,j-1,k)->get_new_v_plus());
        double avg = get_new_v_plus(i,j,k) + get_new_v_plus(i,j-1,k);
        //std::cout<<"yes"<<std::endl;
        set_new_v_plus(i,j,k,avg/2);
        set_new_v_plus(i,j-1,k,avg/2);
      }
    }
    if(getCell(i,j,k+1)->getStatus()!=CELL_EMPTY&& getCell(i,j,k-1)->getStatus()!=CELL_EMPTY){
      if(k-1>-1 && k+1<nz){
        //getCell(i, j,k)->set_new_w_plus(getCell(i,j,k-1)->get_new_w_plus());
        double avg = get_new_w_plus(i,j,k) + get_new_w_plus(i,j,k-1);
        //std::cout<<"yes"<<std::endl;
        set_new_w_plus(i,j,k,avg/2);
        set_new_w_plus(i,j,k-1,avg/2);
      }
    }
  
  }
}