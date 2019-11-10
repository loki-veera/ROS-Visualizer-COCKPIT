var output = document.getElementById("output");
var result = document.getElementById("result");
var node_output = document.getElementById("node_output")
var node_result = document.getElementById("node_result")

// document.querySelector(".container-fluid").style["max-width"] = "500px";
document.getElementById("start").addEventListener("click", initial_run);
document.getElementById("node_start").addEventListener("click",node_test);

function initial_run() {
    var proc = cockpit.script('')
    proc.done(initial_success);
    proc.stream(initial_output);
    proc.fail(initial_fail);

    result.innerHTML = "";
    output.innerHTML = "";
}

function initial_success() {
    result.style.color = "green";
    result.innerHTML = "success";
}

function initial_fail() {
    result.style.color = "red";
    result.innerHTML = "fail";
}

function initial_output(data) {
    output.append(document.createTextNode(data));
}

function node_test() {
    var proc = cockpit.script('')
    proc.done(node_success);
    proc.stream(node_output);
    proc.fail(node_fail);

    node_result.innerHTML = "";
    node_output.innerHTML = "";
}

function node_success() {
    node_result.style.color = "green";
    node_result.innerHTML = "success";
}

function node_fail() {
    node_result.style.color = "red";
    node_result.innerHTML = "fail";
}

function node_output(data) {
    node_output.append(document.createTextNode(data));
}

// Send a 'init' message.  This tells the tests that we are ready to go
cockpit.transport.wait(function() { });
