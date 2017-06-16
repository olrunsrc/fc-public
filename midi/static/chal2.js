$(document).ready(function () {

    $('.srcreq').click(function () {
	d = $(this).data("src")
	sendText('tsk',d);
    });

const tplChkpt = ( v, i) => `
      ${ v>-0.5?(`${i} : ${v.toFixed(3)} </br>`):"" }
`;

const tplPose = ( { rpy, xyz }) => `
      <b>Position:</b> [${xyz[0].toFixed(3)},${xyz[1].toFixed(3)},${xyz[2].toFixed(3)}]
      <b>Euler:</b> [${rpy[0].toFixed(3)},${rpy[1].toFixed(3)},${rpy[2].toFixed(3)}]
      </br>
`;

const tplClock = ( v ) => `
      <b>Sim Time:</b> ${v} </br>
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
      start: ${start.toFixed(3)} </br>
      ${ curr<0.5?"Not Started":"" }
      ${ curr>0.5 & (finished+timedout)<0.5?"Running":"" }
      ${ finished > 0.5?"Finished ":"" }
      ${ timedout>0.5?"Timed Out ":"" }
    </p>
`;

    var wSock;

    $('#connectme').click(function () {
	var ip = $('#ip-data').val();
        var wSockaddr = "ws://"+ip+":9002"
        wSock = new WebSocket(wSockaddr);
        //wSock.binaryType = 'arraybuffer'
        wSock.onmessage = function(event){
        	var showData = $('#show-data');
        	var msg = JSON.parse(event.data);
        	showData.empty();
		if( msg.res ) showData.append("Result received: " + msg.res );
		if( msg.SRC ){
		    $('#pose').html(tplPose(msg.SRC.pose))
		    $('#clock').html(tplClock(msg.SRC.clock))
		    $('#stepcnt').html(`Steps : ${msg.SRC.stepcnt}`)
		    $('#harness').html(`Harness : ${msg.SRC.harness}`)
		    $('#task1').html(tplTask(msg.SRC.task1))
		    $('#task2').html(tplTask(msg.SRC.task2))
		    $('#task3').html(tplTask(msg.SRC.task3))
		    $('#satpitch').html(tplSat(msg.SRC.satpitch))
		    $('#satyaw').html(tplSat(msg.SRC.satyaw))
		    $('#leak').html(`Leak : ${msg.SRC.leak.toFixed(2)}`)
		    $('#score').html(`Score : ${JSON.stringify(msg.SRC.score)}`)
	    	}
        }
    });

    function sendText(cmd,val){
        var msg = { cmd: cmd, x: val };
        wSock.send(JSON.stringify(msg));
        //wSock.send(val);
        }

    $('#ws-src').click(function () {
        var val = $('#ws-data').val();
        sendText('SRC',val);
    });

});

