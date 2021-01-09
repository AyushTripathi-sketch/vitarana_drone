#include<bits/stdc++.h>
using namespace std;
#define pb push_back
#define ll long long

bool binarySearch(vector<int> &a,int start,int end,int val){
    int low =start, high=end-1,mid;
    bool ans = false;
    cout<<"working";
    while(low<=high){
        mid = low + (high-low)/2;
        if(a[mid]==val){
            ans=true;
            break;
        }
        else if(a[mid]>val){
            high = mid-1;
        }
        else{
            low=mid+1;
        }
    }
    return ans;
}

int main(){
    int n,i,j,k;cin>>n;
    int z[n];for(i=0;i<n;i++)   cin>>z[i];
    int c=0; map<int,int> map1; map<int,int> map2;
    for(i=0;i<n;i++){
        //if(z[i]==0) continue;
        for(j=0;j<n;j++){
            for(k=0;k<n;k++){
                auto it1 = map1.find(z[i]*z[j]+z[k]);
                auto it2 = map2.find(z[i]*(z[j]+z[k]));
                if (it1 != map1.end()) {
                    it1->second++;    // increment map's value for key 'c'
                }
                // key not found
                else {
                    map1.insert(pair<int,int>(z[i]*z[j]+z[k],1));
                    }
                if(!(z[i]==0)){
                    if (it2 != map2.end()) {
                        it2->second++;    // increment map's value for key 'c'
                    }
                    // key not found
                    else {
                        map2.insert(pair<int,int>(z[i]*(z[j]+z[k]),1));
                        }
                }
            }
        }
    }
    //cout<<map1.size()<<'\t';
    int count = 0;
    for (auto itr = map1.begin();itr != map1.end(); ++itr){
        auto it = map2.find(itr->first);
        if(it != map2.end()){
            count += (it->second*itr->second);
        }
        //cout<<itr->first<<'\t'<<itr->second<<endl;
    }
    cout<<count;
}
