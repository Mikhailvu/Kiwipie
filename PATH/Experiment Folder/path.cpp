#include <iostream>
#include <fstream>
#include <stdio.h>
#include <limits.h>
#include <tgmath.h>
#include <vector>
#include <algorithm>
#define caps 50
using namespace std;

struct node{
	double x;
	double y;
        int row;
        int column;
};

node minDistance(double dist[caps][caps], bool visit[caps][caps])
{
   // Initialize min value
   int min = INT_MAX;
   node index;
  
   for (int i = 0; i < caps; i++){
      for(int j = 0; j < caps; j++){
         if (visit[i][j] == true && dist[i][j] <= min){
            min = dist[i][j];
            index.row = i;
            index.column = j;
         }
      }
   }
   return index;
}

node dijkstra(bool p[caps][caps], node src, node goal){

     node previous;

     double dist[caps][caps];   // The output array.  dist[i] will hold the shortest
     bool visit[caps][caps];         // distance from src to i
  
     // Initialize all distances as INFINITE and set inaccesible nodes to ped.
     for (int i = 0; i < caps; i++){
        for(int j = 0; j < caps; j++){
           dist[i][j] = INT_MAX; 
           visit[i][j] = p[i][j];
        }
     }
  
     //Distance of source vertex from itself is always 0
     dist[src.row][src.column] = 0;
  
     // Find shortest path for all vertices
   for (int k = 0; k < caps*caps; k++){
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
      node u = minDistance(dist, visit);
  
       // Mark the picked vertex as processed
      visit[u.row][u.column] = false;

      if(u.row == goal.row && u.column == goal.column){
         if(dist[u.row+1][u.column] < dist[u.row+1][u.column+1] && dist[u.row+1][u.column] < dist[u.row][u.column+1] && dist[u.row+1][u.column] < dist[u.row-1][u.column+1] && dist[u.row+1][u.column] < dist[u.row-1][u.column] && dist[u.row+1][u.column] < dist[u.row-1][u.column-1] && dist[u.row+1][u.column] < dist[u.row][u.column-1] && dist[u.row+1][u.column] < dist[u.row+1][u.column-1] && dist[u.row+1][u.column] < dist[u.row][u.column]){
            previous.row = u.row+1;
            previous.column = u.column;
            return previous;
         }else if(dist[u.row+1][u.column+1] < dist[u.row][u.column+1] && dist[u.row+1][u.column+1] < dist[u.row-1][u.column+1] && 
dist[u.row+1][u.column+1] < dist[u.row-1][u.column] && dist[u.row+1][u.column+1] < dist[u.row-1][u.column-1] && dist[u.row+1][u.column+1] < dist[u.row][u.column-1] && dist[u.row+1][u.column+1] < dist[u.row+1][u.column-1] && dist[u.row+1][u.column+1] < dist[u.row][u.column]){
            previous.row = u.row+1;
            previous.column = u.column+1;
            return previous;
         }else if(dist[u.row][u.column+1] < dist[u.row-1][u.column+1] && dist[u.row][u.column+1] < dist[u.row-1][u.column] && dist[u.row][u.column+1] < dist[u.row-1][u.column-1] && dist[u.row][u.column+1] < dist[u.row][u.column-1] && dist[u.row][u.column+1] < dist[u.row+1][u.column-1] && dist[u.row][u.column+1] < dist[u.row][u.column]){
            previous.row = u.row;
            previous.column = u.column+1;
            return previous;
         }else if(dist[u.row-1][u.column+1] < dist[u.row-1][u.column] && dist[u.row-1][u.column+1] < dist[u.row-1][u.column-1] && dist[u.row-1][u.column+1] < dist[u.row][u.column-1] && dist[u.row-1][u.column+1] < dist[u.row+1][u.column-1] && dist[u.row-1][u.column+1] < dist[u.row][u.column]){
            previous.row = u.row-1;
            previous.column = u.column+1;
            return previous;
         }else if(dist[u.row-1][u.column] < dist[u.row-1][u.column-1] && dist[u.row-1][u.column] < dist[u.row][u.column-1] && dist[u.row-1][u.column] < dist[u.row+1][u.column-1] && dist[u.row-1][u.column] < dist[u.row][u.column]){
            previous.row = u.row-1;
            previous.column = u.column;
            return previous;
         }else if(dist[u.row-1][u.column-1] < dist[u.row][u.column-1] && dist[u.row-1][u.column-1] < dist[u.row+1][u.column-1] && dist[u.row-1][u.column-1] < dist[u.row][u.column]){
            previous.row = u.row-1;
            previous.column = u.column-1;
            return previous;
         }else if(dist[u.row][u.column-1] < dist[u.row+1][u.column-1] && dist[u.row][u.column-1] < dist[u.row][u.column]){
            previous.row = u.row;
            previous.column = u.column-1;
            return previous;
         }else if(dist[u.row+1][u.column-1] < dist[u.row][u.column]){
            previous.row = u.row+1;
            previous.column = u.column-1;
            return previous;
         }else{
            previous.row = u.row;
            previous.column = u.column;
            return previous;
         }
         /*if(!visit[u.row-1][u.column-1] && p[u.row-1][u.column-1] &&u.row-1 > -1 && u.column-1 > -1){
            previous.row = u.row-1;
            previous.column = u.column-1;
            return previous;
         }else if(!visit[u.row-1][u.column+1] && p[u.row-1][u.column+1] &&u.row-1 > -1 && u.column+1 < caps){
            previous.row = u.row-1;
            previous.column = u.column+1;
            return previous;
         }else if(!visit[u.row+1][u.column-1] && p[u.row+1][u.column-1] &&u.row+1 < caps && u.column-1 > -1){
            previous.row = u.row+1;
            previous.column = u.column-1;
            return previous;
         }else if(!visit[u.row+1][u.column+1] && p[u.row+1][u.column+1] &&u.row+1 < caps && u.column+1 < caps){
            previous.row = u.row+1;
            previous.column = u.column+1;
            return previous;
         }else if(!visit[u.row][u.column-1] && p[u.row][u.column-1] &&u.column-1 > -1){
            previous.row = u.row;
            previous.column = u.column-1;
            return previous;
         }else if(!visit[u.row][u.column+1] && p[u.row][u.column+1] &&u.column+1 < caps){
            previous.row = u.row;
            previous.column = u.column+1;
            return previous;
         }else if(!visit[u.row-1][u.column] && p[u.row-1][u.column] && u.row-1 > -1){
            previous.row = u.row-1;
            previous.column = u.column;
            return previous;
         }else if(!visit[u.row+1][u.column] && p[u.row+1][u.column] && u.row+1 < caps){
            previous.row = u.row+1;
            previous.column = u.column;
            return previous;
         }else{
            previous.row = u.row;
            previous.column = u.column;
            return previous;
         }*/
      }
       
       // Update dist value of the adjacent vertices of the picked vertex.
      
      if( visit[u.row][u.column+1] && u.column+1 < caps && (dist[u.row][u.column]+1) < dist[u.row][u.column+1]){
         dist[u.row][u.column+1] = dist[u.row][u.column]+1;
         //dist[u.row][u.column+1] = sqrt(pow(u.row-src.row,2)+pow(u.column+1-src.column,2));
      }

      if( visit[u.row][u.column-1] && u.column-1 > -1 && (dist[u.row][u.column]+1) < dist[u.row][u.column-1]){
         dist[u.row][u.column-1] = dist[u.row][u.column]+1;
         //dist[u.row][u.column-1] = sqrt(pow(u.row-src.row,2)+pow(u.column-1-src.column,2));
      }

      if( visit[u.row+1][u.column] && u.row+1 < caps && (dist[u.row][u.column]+1) < dist[u.row+1][u.column]){
         dist[u.row+1][u.column] = dist[u.row][u.column]+1;
         //dist[u.row+1][u.column] = sqrt(pow(u.row+1-src.row,2)+pow(u.column-src.column,2));
      }

      if( visit[u.row-1][u.column] && u.row-1 > -1 && (dist[u.row][u.column]+1) < dist[u.row-1][u.column]){
         dist[u.row-1][u.column] = dist[u.row][u.column]+1;
         //dist[u.row-1][u.column] = sqrt(pow(u.row-1-src.row,2)+pow(u.column-src.column,2));
      }

      if( visit[u.row-1][u.column-1] && u.row-1 > -1 && u.column-1 > -1 && (dist[u.row][u.column]+sqrt(2)) < dist[u.row-1][u.column-1]){
         dist[u.row-1][u.column-1] = dist[u.row][u.column]+sqrt(2);
         //dist[u.row-1][u.column-1] = sqrt(pow(u.row-1-src.row,2)+pow(u.column-1-src.column,2));
      }

      if( visit[u.row-1][u.column+1] && u.row-1 > -1 && u.column+1 < caps && (dist[u.row][u.column]+sqrt(2)) < dist[u.row-1][u.column+1]){
         dist[u.row-1][u.column+1] = dist[u.row][u.column]+sqrt(2);
         //dist[u.row-1][u.column+1] = sqrt(pow(u.row-1-src.row,2)+pow(u.column+1-src.column,2));
      }

      if( visit[u.row+1][u.column-1] && u.row+1 < caps && u.column-1 > -1 && (dist[u.row][u.column]+sqrt(2)) < dist[u.row+1][u.column-1]){
         dist[u.row+1][u.column-1] = dist[u.row][u.column]+sqrt(2);
         //dist[u.row+1][u.column-1] = sqrt(pow(u.row+1-src.row,2)+pow(u.column-1-src.column,2));
      }

      if( visit[u.row+1][u.column+1] && u.row+1 < caps && u.column+1 < caps && (dist[u.row][u.column]+sqrt(2)) < dist[u.row+1][u.column+1]){
         dist[u.row+1][u.column+1] = dist[u.row][u.column]+sqrt(2);
         //dist[u.row+1][u.column+1] = sqrt(pow(u.row+1-src.row,2)+pow(u.column+1-src.column,2));
      }
   }
}

int main(int argc, char* argv[]){
    
    //Takes the (x,y) position for point A(start point) and point B(end point). If not sufficient arguments are provided,
    //return an errormessage.
    if(argc < 5){
       std::cerr << "Not enough input argument" << std::endl;
       return 1;
    }

    node src;
    node goal;

    //Assign variables with input arguments
    src.x =  std::stod(argv[1]);
    src.y =  std::stod(argv[2]);
    goal.x =  std::stod(argv[3]);
    goal.y = std::stod(argv[4]);
   
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


    //Initialize the caps of our grid. The amount of nodes are equal to caps².
    double x_pos[caps];
    double y_pos[caps];

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

    for(int i = 0; i<caps;i++){
       x_pos[i] = -largest_value + largest_value*2*(i+1)/(caps+1);
    }

    for(int j = 0; j<caps;j++){
       y_pos[j] = -largest_value + largest_value*2*(j+1)/(caps+1);
    }

    

    bool p[caps][caps];

    //Print out the x_pos and y_pos of the nodes
    //for(int i = 0; i < caps; i++){
    //   std::cout << x_pos[i] << " " << y_pos[i] << std::endl; 
    //} 

    // If any nodes are too close to the wall. Set it to 0. Otherwise it is set to 1.
    for(int i = 0; i <caps; i++){
       for(int j = 0; j <caps; j++){
          for(int k = 0; k < number_of_lines; k++){
             if(x_min[k] < x_pos[j] && x_pos[j] < x_max[k] && y_min[k] < y_pos[i] && y_pos[i] < y_max[k]){
                p[i][j] = false;
                break;
             }else{
                p[i][j] = true;
             }
          }
       }
    }

    double distance = INT_MAX;

    for(int i = 0; i < caps; i++){
       for(int j = 0; j < caps; j++){
          if(sqrt(pow(x_pos[j]-src.x,2) + pow(y_pos[i]-src.y,2)) < distance){
              distance = sqrt(pow(x_pos[j]-src.x,2) + pow(y_pos[i]-src.y,2));
              src.row = i;
              src.column = j;
          }
       }
    }

    distance = INT_MAX;

    for(int i = 0; i < caps; i++){
       for(int j = 0; j < caps; j++){
          if(sqrt(pow(x_pos[j]-goal.x,2) + pow(y_pos[i]-goal.y,2)) < distance){
              distance = sqrt(pow(x_pos[j]-goal.x,2) + pow(y_pos[i]-goal.y,2));
              goal.row = i;
              goal.column = j;
          }
       }
    }

    

    std::vector<node> n;
    n.push_back(goal);
    for(int i = 0; i < caps*caps; i++){
       if(src.row == n[i].row && src.column == n[i].column){
          break;
       }else{
          n.push_back(dijkstra(p,src,n[i]));
       }
    }

    std::reverse(n.begin(), n.end());
    
    for(int i = 0; i<n.size(); i++){
       std::cout << n[i].row << " " << n[i].column << std::endl;
    }
    std::cout << "Start nodes: " << src.row << " " << src.column << std::endl;
    std::cout << "Goal nodes: " << goal.row << " " << goal.column << std::endl;

    //Print out the nodes in the terminal  
    for(int i = caps-1; i>=0 ;i--){
       for(int j = 0; j < caps; j++){
          if(src.row == i && src.column == j){
             std::cout << "S";
          }else if(goal.row == i && goal.column == j){
             std::cout << "G";
          }else if(p[i][j]){
             for(int k = 0; k<n.size();k++){
                if(n[k].row == i && n[k].column == j){
                   std::cout << "X";
                   break;
                }
                if(k == n.size()-1){
                   std::cout << " " ;
                }
             }
          }else{
             std::cout << "*" ;
          }
       }
       std::cout << std::endl;
    }

    for(int i = 0; i<n.size(); i++){
       n[i].x = -largest_value + largest_value*2*(n[i].column+1)/(caps+1);
       n[i].y = -largest_value + largest_value*2*(n[i].row+1)/(caps+1);
       std::cout << n[i].x << " " << n[i].y << std::endl;
    }


    return 0;
}
