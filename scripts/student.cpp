#include <bits/stdc++.h>
using namespace std;
class Student 
{
public:
    string name;   
    Student(string s){
        name=s;
    } 
    Student(){
        name="Unknown";
    } 
}; 
  
int main() 
{ 
   Student s1("Ayush"); // p1 is a object
   Student s2;
   cout<<s1.name<<endl;
   cout<<s2.name<<endl; 
} 
