$(document).ready(function () {

    $('.srcreq').click(function () {
	d = $(this).data("src").split(';');
        msg = { cmd: d[0], r: d[1] };
	if(d.length>2) msg['t'] = d[2];
        wSock.send(JSON.stringify(msg));
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

    var lochn = location.hostname
    var wshn = lochn?lochn:"localhost"
    //var wSockaddr = "ws://" + location.hostname?location.hostname:"localhost" + ":9000"
    //var wSockaddr = "ws://192.168.0.61:9001"
    //var wSockaddr = "ws://192.168.0.79:9001"
    var wSockaddr = "ws://127.0.0.1:9001"
    var wSock = new WebSocket(wSockaddr);
    wSock.binaryType = 'arraybuffer'
    function sendText(cmd,val){
        var msg = { cmd: cmd, x: val };
        wSock.send(JSON.stringify(msg));
        //wSock.send(val);
        }

    wSock.onmessage = function(event){
        var showData = $('#show-data');
	var data = event.data
	var sz = data.byteLength
        var hd = new Int32Array( data.slice(0,8) )
        var ld = new Float32Array( data.slice(8,28) )
	var img = new Uint8Array( data.slice(28,hd[0]) )
	//var ldstr = ld.map(function(v){return v.toFixed(2)}).join(', ')
	var ldstr = []
	ld.forEach(function(v,i,a){ ldstr.push(v.toFixed(3)) })
        showData.empty();
	showData.append('Message length is ' + sz.toString() + "<br/>")
	showData.append(hd[0].toString() + ', ' + hd[1].toString() + ', ')
	//showData.append(ld[0].toString() + ', ' + ld[1].toString() + ', ')
	//showData.append(ld[2].toString() + ', ' + ld[3].toString() + ', ')
	//showData.append(ld[4].toString() + "<br/>" )
	showData.append(ldstr.join(', ') + "<br/>" )
	showData.append('Array length is ' + img.length.toString() + "<br/>")
        var canvas = document.getElementById('mycam');
	var ctx = canvas.getContext('2d');
	var imgd = ctx.createImageData(canvas.width,canvas.height);
	var imgdata = imgd.data
        var ilen = imgdata.length-4
	//var imgdata = ctx.getImageData(0,0,1024,544).data;
	for( var i=0; i < parseInt(hd[0]/3); i++ ){
	    //imgdata[ 4*i    ] = img[ 3*i + 2];
	    //imgdata[ 4*i + 1] = img[ 3*i + 1];
	    //imgdata[ 4*i + 2] = img[ 3*i + 0];
	    //imgdata[ 4*i + 3] = 255;
	    imgdata[ ilen - 4*i    ] = img[ 3*i + 2];
	    imgdata[ ilen - 4*i + 1] = img[ 3*i + 1];
	    imgdata[ ilen - 4*i + 2] = img[ 3*i + 0];
	    imgdata[ ilen - 4*i + 3] = 255;
	    }
	ctx.putImageData(imgd,0,0)
        }

});

