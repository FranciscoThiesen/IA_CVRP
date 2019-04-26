#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <stack>
#include <functional>
#include <utility>
#include <cstdio>
#include <cassert>
#include <map>
#include <unordered_map>
#include <set>
#include <random>
#include <ctime>
#include <deque>
#include "time_lib.h"
using namespace std;

#define TRACE(x) x
#define WATCH(x) TRACE( cout << #x" = " << x << endl)
#define PRINT(x) TRACE(printf(x))
#define WATCHR(a, b) TRACE( for(auto c = a; c != b;) cout << *(c++) << " "; cout << endl)
#define WATCHC(V) TRACE({cout << #V" = "; WATCHR(V.begin(), V.end()); } )

#define all(v) (v).begin(), (v).end()
#define sz(v) (int) (v).size()

#define rep(i,a,b) for(int (i) = (a); (i) < (b); ++(i))

#define pb push_back
#define mp make_pair
#define fi first
#define se second
#define shandom_ruffle random_shuffle

using ll = long long;
using vi = vector<int>;
using vll = vector<ll>;

constexpr int INF = 0x3f3f3f3f;
constexpr long long MOD = 10000000007;

inline ll modPow( ll a, ll b, ll mod = MOD) {
    ll res = 1; a %= mod; assert(b >= 0);
    for(;b;b>>=1) {
        if(b&1) res = (res * a) % mod;
        a = (a * a) % mod;
    }
    return res;
}

int main()
{
    ios::sync_with_stdio(false); cin.tie(NULL); 
    int val = 13;

    clock_t ST = get_time();
    for(int i = 0; i < 123123; ++i) {
        val = (val * val % 13) + val;
    }
    clock_t ND = get_time();
    
    long double duration = time_in_ms(ST, ND);
    
    return 0;
}

