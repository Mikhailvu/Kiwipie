#include <iostream>
#include <fstream>
#include <stdio.h>
#include <limits.h>
using namespace std;


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

    //Print out the number of lines in the txt file.
    //std::cout << "Number of lines in text file: " << number_of_lines << std::endl;

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

    //Create the intervals for the walls
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

    //print the intervall for the walls.
    //for(int i = 0 ; i<number_of_lines;i++){
    //   std::cout << " xmin = " << x_min[i] << " xmax = " << x_max[i] << " ymin = " << y_min[i] << " ymax = " << y_max[i] << std::endl;
    //}


    //Initialize the size of our grid. The amount of nodes are equal to size².
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

    //Print out the x_pos and y_pos of the nodes
    //for(int i = 0; i < size; i++){
    //   std::cout << x_pos[i] << " " << y_pos[i] << std::endl; 
    //} 

    // If any nodes are too close to the wall. Set it to 0. Otherwise it is set to 1.
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
// find start and goal
int i_start;
int j_start;
int i_end;
int j_end;


for (int i = 0; i <size-1; i++) {
if (x_src >= x_pos[i] && x_src < x_pos[i + 1])
{if(x_src - x_pos[i] >= x_pos[i+1] - x_src){
    j_start = i + 1;
} else {
	j_start = i;}
}
}

for (int i = 0; i <size-1; i++) {
if (y_src >= y_pos[i] && y_src < y_pos[i + 1])
{if(y_src - y_pos[i] >= y_pos[i+1] - y_src){
    i_start = i + 1;
}       else {
	i_start = i;}
}
}

for (int i = 0; i <size-1; i++) {
if (x_goal >= x_pos[i] && x_goal < x_pos[i + 1])
{if(x_goal - x_pos[i] >= x_pos[i+1] - x_goal){
    j_end = i + 1;
}   else {
	j_end = i;}
}
}

for (int i = 0; i <size-1; i++) {
if (y_goal >= y_pos[i] && y_goal < y_pos[i + 1])
{if(y_goal - y_pos[i] >= y_pos[i+1] - y_goal){
    i_end = i + 1;
}       else {
	i_end = i;}
}
}

// set start and endpoints in grid
p[i_start][j_start]=2;
p[i_end][j_end]=3;

}


    //Print out the nodes in the terminal  
    for(int i = size-1; i>=0 ;i--){
       for(int j = 0; j < size; j++){
          std::cout << p[i][j];
       }
       std::cout << std::endl;
    }

    return 0;
}
