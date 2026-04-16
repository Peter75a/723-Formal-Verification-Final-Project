bool in_cs[2];

active [2] proctype P() {
    do
    :: atomic {
         !in_cs[1-_pid] ->
         in_cs[_pid] = true;
         assert(!(in_cs[0] && in_cs[1]));
         in_cs[_pid] = false;
       }
    od
}