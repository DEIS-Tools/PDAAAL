# PDAAAL
PushDown Automata - AALborg

## Dependencies
Install dependencies (tested on Ubuntu 21.10): [TODO: Actually test this]
````shell
sudo apt install build-essentials cmake
````

Build using cmake:
````bash
mkdir build 
cd build
cmake ..
make
````

## Input format
For pushdown systems we have two similar JSON formats. The only difference is whether states are named or indexed. 

Indexed states:
````json
{
  "pda": {
    "states": [
      {"A": {"to": 1, "pop": ""},
       "B": {"to": 0, "push": "A"}},
      {"A": [
        {"to": 0, "swap": "A"},
        {"to": 2, "push": "B"}]},
      {"B": {"to": 2, "pop": ""}}
    ]
  }
}
````

Names states:
````json
{
  "pda": {
    "states": {
      "p0": {
        "A": {"to": "p1", "pop": ""}, 
        "B": {"to": "p0", "push": "A"}}, 
      "p1": {
        "A": [
          {"to": "p0", "swap": "A"}, 
          {"to": "p2", "push": "B"}]}, 
      "p2": {
        "B": {"to": "p2", "pop": ""}}
    }
  }
}
````
