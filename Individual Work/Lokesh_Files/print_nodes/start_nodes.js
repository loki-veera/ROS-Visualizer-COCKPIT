var express = require("express");
var app = express();
var path = require("path");
var raw_data = "";
var bodyParser = require('body-parser')
app.get('/', function(req, res){
    console.log('index_nodes.html')
    res.sendFile(path.join(__dirname+'/index_nodes.html'))
});
app.listen(3000, function(){
    console.log('server running on port 3000');
});

app.post('/up', function(req, res){
    var spawn = require("child_process").spawn;
    var process = spawn('python',["./start_nodes.py"]);
    process.stdout.on('data', function(data){
        //console.log("Data ");
        raw_data = data.toString();
        //raw_data = raw_data.slice(1,-2).replace(/\'/g,"").split(",");
       // var_values = Object.values(raw_data);
        //for (var x of var_values){
        //    console.log(x);
        //}
        //res.set('Content-Type', 'text/html');
        //res.send(raw_data);
        res.render('test.html',{myVar:raw_data})
    });
}); 