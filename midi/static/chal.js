$(document).ready(function () {
    $('#get-data').click(function () {
        var showData = $('#show-data');

        $.getJSON('/step/14', function (data) {
            console.log(data);

            var items = data.items.map(function (item) {
              return item.key + ': ' + item.value;
              });

            showData.empty();

            if (items.length) {
                var content = '<li>' + items.join('</li><li>') + '</li>';
                var list = $('<ul />').html(content);
                showData.append(list);
            }
        });

        showData.text('Loading the JSON file.');
    });

    $('#post-step').click(function () {
        var showData = $('#show-data');
	var val = $('#ws-data').val();

        $.post('/step/1',
            { x: val,
              n: 1
            },
            function (data, status) {
                console.log(data);
                var items = "Data: " + data + " Status: " + status;
                showData.empty();
                showData.append(items);
            });
    });

    $('.srcreq').click(function () {
	d = $(this).data("src")
	sendText('tsk',d);
/*        $.post('/step/2',
            { msg: d },
            function (data, status) {
                console.log(data);
                var items = "Data: " + data + " Status: " + status;
                showData.empty();
                showData.append(items);
            });
*/
    });

const tplChkpt = ( v, i) => `
      ${ v>-0.5?(`${i} : ${v.toFixed(3)} </br>`):"" }
`;

const tplSat = ( { cur, tgt, dir, ok, done } ) => `
      <b>Satellite ${ dir>0.5?"Yaw":"Pitch" }</b> </br>
      Target : ${ tgt.toFixed(3)} </br>
      Current : ${ cur.toFixed(3)} </br>
      ${ ok>0.5?"OK </br>":"" }
      ${ done>0.5?"Done </br>":"" }
`;

const tplTask = ({ curr, elapsed, task, comp, timedout, start, finished }) => `
    <p class="xtask">
      <b>task: ${task.toFixed(0)} </b></br>
      elapsed: ${elapsed.toFixed(3)} </br>
      curr: ${curr.toFixed(0)} </br>
      ${ comp.map(tplChkpt).join('') } </br>
      start: ${elapsed.toFixed(3)} </br>
      ${ curr<0.5?"Not Started":"" }
      ${ curr>0.5 & (finished+timedout)<0.5?"Running":"" }
      ${ finished > 0.5?"Finished ":"" }
      ${ timedout>0.5?"Timed Out ":"" }
    </p>
`;

    //var wSockaddr = "ws://" + location.hostname + ":9002"
    //var wSockaddr = "ws://10.9.20.1:9002"
    //var wSockaddr = "ws://192.168.2.10:9002"
    var wSockaddr = "ws://127.0.0.1:9002"
    var wSock = new WebSocket(wSockaddr);
    function sendText(cmd,val){
        var msg = { cmd: cmd, x: val };
        wSock.send(JSON.stringify(msg));
        //wSock.send(val);
        }

    wSock.onmessage = function(event){
        var showData = $('#show-data');
        var msg = JSON.parse(event.data);
        showData.empty();
	if( msg.res ) showData.append("Result received: " + msg.res );
	if( msg.SRC ){
	    $('#task1').html(tplTask(msg.SRC.task1))
	    $('#task2').html(tplTask(msg.SRC.task2))
	    $('#task3').html(tplTask(msg.SRC.task3))
	    $('#satpitch').html(tplSat(msg.SRC.satpitch))
	    $('#satyaw').html(tplSat(msg.SRC.satyaw))
	    $('#leak').html(`Leak : ${msg.SRC.leak.toFixed(2)}`)
	    }
        }

    $('#ws-square').click(function () {
        var val = $('#ws-data').val();
	//alert(val);
        sendText('sqr',val);
    });

    $('#ws-src').click(function () {
        var val = $('#ws-data').val();
        sendText('SRC',val);
    });

});

