# PDAAAL
PushDown Automata - AALborg

## Dependencies
Install dependencies:
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
### Pushdown Systems
For pushdown systems we have two similar JSON formats. The only difference is whether states are named or indexed. Each state has a dictionary, where keys correspond to precondition labels and the values are the rules that can be applied if the top of the stack matches the label. A rule indicates the "to" state and the push, swap or pop operation. For weighted pushdown systems, the rule also has a weight attribute. 

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
### P-automata
P-automata are defined relative to a pushdown system. 
For the PDS above with named states, an example of a P-automaton is: 
````json
{
  "P-automaton": {
    "accepting": [3],
    "initial": ["p0","p1","p2"],
    "edges": [
      ["p0","A",3],
      ["p0","B",4],
      [4,"A",3],
      ["p1","A",3]
    ]
  }
}
````

It is also possible to define a P-automaton using regular expressions to describe the stack:
````
< [p0], [B]? [A] > | < [p1], [A] >
````

### Reachability Problem Instance
For convenience, we can group a pushdown system and two P-automata (representing initial and final configurations) together into a reachability problem instance. This is a JSON array, where the first element has meta-data of whether PDS states are named and the type of weight used. After this follows the PDS, the initial automaton and the final automaton. 
````json
{
  "instance": [
    {"state-names": true, "weight-type": "uint"},
    { "states": {
        "p0": {
          "A": {"to": "p1", "pop": "", "weight": 1},
          "B": {"to": "p0", "push": "A", "weight": 0}},
        "p1": {
          "A": [
            {"to": "p0", "swap": "A", "weight": 2},
            {"to": "p2", "push": "B", "weight": 1}]},
        "p2": {
          "B": {"to": "p2", "pop": "", "weight": 3}}
      }
    },
    { "accepting": [3],
      "edges": [
        ["p0","A",3],
        ["p0","B",4],
        [4,"A",3],
        ["p1","A",3]
      ]
    },
    { "accepting": ["p2",3],
      "edges": [
        ["p2","A",3]
      ]
    }
  ]
}
````

## Running PDAAAL
To run PDAAAL, provide an instance file for the option ```--input```, and select the engine ```-e 1```(post*), ```-e 2```(pre*), or ```-e 3```(dual*) and the trace type ```-t 0```(no trace), ```-t 1```(any trace), ```-t 2```(shortest trace), or ```-t 3```(longest trace). For finding shortest trace with negative weights use ```-t 4``` to enable the fixed-point algorithm for shortest trace.

Example of using the post* algorithm to find the shortest trace in the above example:
````bash
./bin/pdaaal --input example-instance.json -e 1 -t 2
````
The output of running the tool contains the boolean reachability result, the weight, and a trace along with some timing information. For example the above command can produce this output:
````json
{
    "parsing-duration" : 0.0542163,
    "engine" : "post*",
    "weight" : 4,
    "rtime" : 0.0070728,
    "result" : true,
    "trace" : [{"stack":["A"],"state":1},{"stack":["B","A"],"state":2},{"stack":["A"],"state":2}]
}
````
