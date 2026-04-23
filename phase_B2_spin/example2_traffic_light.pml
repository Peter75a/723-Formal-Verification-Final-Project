mtype = { NS_GREEN, EW_GREEN }

mtype light;

init {
    light = NS_GREEN;
    run Controller();
    run Check();
}

proctype Controller() {
    do
    :: light = NS_GREEN
    :: light = EW_GREEN
    od
}

proctype Check() {
    do
    :: assert(light == NS_GREEN || light == EW_GREEN)
    od
}