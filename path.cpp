#include <iostream>
#include <fstream>
#include <stdio.h>
#include <limits.h>
using namespace std;

/*bool Intersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4){
}*/



int main(int argc, char* argv[]){
    
    //Takes the (x,y) position for point A(start point) and point B(end point). If not sufficient arguments are provided,
    //return an errormessage.
    if(argc < 5){
       std::cerr << "Not enough input argument" << std::endl;
       return 1;
    }

    //Assign variables with input arguments
    double x_src =  std::stod(argv[1]);
    double y_src =  std::stod(argv[2]);
    double x_goal =  std::stod(argv[3]);
    double y_goal = std::stod(argv[4]);
   
    //This whole codeblock stores the x and y positions from a txt file into two position vectors. x[] and y[].
    int number_of_lines = 0;
    std::string line;
    std::ifstream myfile("simulation-map.txt");
 
    while (std::getline(myfile, line)){
        ++number_of_lines;
    }

    std::cout << "Number of lines in text file: " << number_of_lines << std::endl;


    double x[2*number_of_lines];
    double y[2*number_of_lines];
    double sd = 0.15;

    char fill;

    std::ifstream myfile1("simulation-map.txt");
 
    for(int i = 0; i<2*number_of_lines;i++){
    myfile1 >> x[i];
    myfile1 >> fill;
    myfile1 >> y[i];
    myfile1 >> fill;
    }

    double x_min[number_of_lines];
    double y_min[number_of_lines];
    double x_max[number_of_lines];
    double y_max[number_of_lines];

    for(int i = 0; i < number_of_lines;i++){
       if(x[2*i] < x[2*i+1]){
          x_min[i] = x[2*i]-sd;
          x_max[i] = x[2*i+1]+sd; 
       }else{
          x_min[i] = x[2*i+1]-sd;
          x_max[i] = x[2*i]+sd;
       }
       if(y[2*i] < y[2*i+1]){
          y_min[i] = y[2*i]-sd;
          y_max[i] = y[2*i+1]+sd;
       }else{
          y_min[i] = y[2*i+1]-sd;
          y_max[i] = y[2*i]+sd;
       }
    }

    for(int i = 0 ; i<number_of_lines;i++){
       std::cout << " xmin = " << x_min[i] << " xmax = " << x_max[i] << " ymin = " << y_min[i] << " ymax = " << y_max[i] << std::endl;
    }


    //Initialize how many nodes will exist in our grid. rows are the amount of rows in our grid and column is the amount of column in our grid.
    // The grid is represented by 2 vectors x_pos and y_pos and creates a squared grid. x_pos[0] and y_pos[0] represent the position in the
    // southleft corner and x_pos[rows-1] and y_pos[columns-1] represents the position in the northeast corner.
    int size = 15;
    double x_pos[size-1];
    double y_pos[size-1];

    double largest_value = 0;
    for(int i = 0; i<2*number_of_lines; i++){
       if(x[i]>largest_value){
          largest_value = x[i];
       }
       if(-1*x[i]>largest_value){
          largest_value = -1*x[i];
       }
       if(y[i]>largest_value){
          largest_value = y[i];
       }
       if(-1*y[i]>largest_value){
          largest_value = -1*y[i];
       }
    }

    for(int i = 0; i<size;i++){
       x_pos[i] = -largest_value + largest_value*2*(i+1)/(size+1);
    }

    for(int j = 0; j<size;j++){
       y_pos[j] = -largest_value + largest_value*2*(j+1)/(size+1);
    }

    

    int p[size-1][size-1];

    for(int i = 0; i < size; i++){
       std::cout << x_pos[i] << " " << y_pos[i] << std::endl; 
    } 

    for(int i = 0; i <size; i++){
       for(int j = 0; j <size; j++){
          for(int k = 0; k < number_of_lines; k++){
             
             if(x_min[k] < x_pos[j] && x_pos[j] < x_max[k] && y_min[k] < y_pos[i] && y_pos[i] < y_max[k]){
                p[i][j] = 0;
                break;
             }else{
                p[i][j] = 1;
             }
          }
       }
    }
    
    
  
    for(int i = 0; i<size;i++){
       for(int j = 0; j < size; j++){
          std::cout << p[i][j];
       }
       std::cout << std::endl;
    }

    return 0;
}
