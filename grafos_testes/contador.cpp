#include <iostream>
#include <vector>
using namespace std;
int main(){

  vector<int> vetor;
  int num;
  int cont = 0;
  while (cin >> num){
    cin >> num;
    if (num == 1){
      cont++;
    }
      cout << cont;

    vetor.push_back(num);
  }
  cout << vetor.size();


  return 0;
}