$(document).ready(function () {

    $('.srcreq').click(function () {
	d = $(this).data("src").split(';');
        msg = { cmd: d[0], r: d[1] };
	if(d.length>2) msg['t'] = d[2];
        wSock.send(JSON.stringify(msg));
    });

    var wSock;

    $('#connectme').click(function () {
	var ip = $('#ip-data').val();
        var wSockaddr = "ws://"+ip+":9001"
        wSock = new WebSocket(wSockaddr);
        wSock.binaryType = 'arraybuffer'
        wSock.onmessage = function(event){
		var showData = $('#show-data');
		var data = event.data
		var sz = data.byteLength
		if( sz > 100){
			var hd = new Int32Array( data.slice(0,8) )
			var ld = new Float32Array( data.slice(8,28) )
			var img = new Uint8Array( data.slice(28,hd[0]) )
			var ldstr = []
			ld.forEach(function(v,i,a){ ldstr.push(v.toFixed(3)) })
			showData.empty();
			showData.append('Message length is ' + sz.toString() + "<br/>")
			showData.append(hd[0].toString() + ', ' + hd[1].toString() + ', ')
			showData.append(ldstr.join(', ') + "<br/>" )
			if( sz == 6556) showSmall( img, hd[1] )
			if( sz == 34844) showGray( img, hd[1] )
			if( sz == 2204) showSeg( img, hd[1] )
		} else {
			var img = new Uint8Array( data.slice(28,sz) )
			showData.append('<br/>' + String.fromCharCode.apply(null,img)) 
		}
        }
    });

    function sendText(cmd,val){
        var msg = { cmd: cmd, x: val };
        wSock.send(JSON.stringify(msg));
        //wSock.send(val);
        }

    const rgbhlp = (i,m) => {return Math.floor(256*(m*i+0.3125))}
    const rgbname = (hue) => {
		r = rgbhlp(Math.floor(hue/8),.375);
                g = rgbhlp(Math.floor(hue/2)%4,.1875);
                b = rgbhlp(hue%2,.375);
		return [r, g, b];
 }

    const showSeg = (img,num) => {
        //we know this is 34x64 mono 2kB coded
	$('#show-data').append('Show seg length: ' + img.length.toString() + "<br/>");
	$('#seglabel').html('<b>#' + num + "</b><br/>");
	var mycanvas = document.getElementById('segimg');
	var destCtx = mycanvas.getContext('2d');
        destCtx.fillStyle = 'green';
	destCtx.fillRect( 0, 0, mycanvas.width, mycanvas.height);
	var imgd = destCtx.createImageData(mycanvas.width,mycanvas.height);
	var imgdata = imgd.data
	var olen = imgdata.length-4
 	var inlen = img.length
	var resize = [34,64,4,1] //height, width, scale>1, depth
	for( var y=0; y < resize[0]; y++ ){
	  for( var x=0; x < resize[1]; x++ ){
            inidx = resize[1]*y + x 
            outidx = resize[2]*(resize[1]*resize[2]*y + x)
	    pixel = img[inidx]
            code = rgbname(pixel)
            d0 = code[0];
	    d1 = code[1];
	    d2 = code[2];
	    d3 = 255;
	    for( var j=0; j < resize[2] ; j++ ){
	      for( var i=0; i < resize[2]; i++ ){
                idx = outidx + resize[1]*resize[2]*j + i
	        imgdata[ 4*idx    ] = d0;
	        imgdata[ 4*idx + 1] = d1;
	        imgdata[ 4*idx + 2] = d2;
	        imgdata[ 4*idx + 3] = d3;
              }
	    }
          }
        }
	destCtx.putImageData(imgd,0,0)
    }

    const showSmall = (img,num) => {
        //we know this is 34x64 rgb 7kB
	$('#show-data').append('Show small length: ' + img.length.toString() + "<br/>");
	$('#smalllabel').html('<b>#' + num + "</b><br/>");
	var mycanvas = document.getElementById('smallimg');
	var destCtx = mycanvas.getContext('2d');
        destCtx.fillStyle = 'green';
	destCtx.fillRect( 0, 0, mycanvas.width, mycanvas.height);
	var imgd = destCtx.createImageData(mycanvas.width,mycanvas.height);
	var imgdata = imgd.data
	var olen = imgdata.length-4
 	var inlen = img.length
        var resize = [34,64,4,3] //height, width, scale>1, depth
        resizergb(img,imgdata,resize)
	destCtx.putImageData(imgd,0,0)
    }

    const showGray = (img,num) => {
        //we know this is 136x256 mono 34kB
	$('#show-data').append('Show gray length: ' + img.length.toString() + "<br/>");
	$('#graylabel').html('<b>#' + num + "</b><br/>");
	var mycanvas = document.getElementById('grayimg');
	var destCtx = mycanvas.getContext('2d');
        destCtx.fillStyle = 'green';
	destCtx.fillRect( 0, 0, mycanvas.width, mycanvas.height);
	var imgd = destCtx.createImageData(mycanvas.width,mycanvas.height);
	var imgdata = imgd.data
	var olen = imgdata.length-4
 	var inlen = img.length
	for( var i=0; i < inlen; i++ ){
	    imgdata[ 4*i    ] = img[i];
	    imgdata[ 4*i + 1] = img[i];
	    imgdata[ 4*i + 2] = img[i];
	    imgdata[ 4*i + 3] = 255;
	    }
	destCtx.putImageData(imgd,0,0)
    }

    const resizergb = (inimg,outimg,resize) => {
	for( var y=0; y < resize[0]; y++ ){
	  for( var x=0; x < resize[1]; x++ ){
            inidx = resize[1]*y + x 
            outidx = resize[2]*(resize[1]*resize[2]*y + x)
            d2 = inimg[ 3*inidx + 0];
	    d1 = inimg[ 3*inidx + 1];
	    d0 = inimg[ 3*inidx + 2];
	    d3 = 255;
	    for( var j=0; j < resize[2] ; j++ ){
	      for( var i=0; i < resize[2]; i++ ){
                idx = outidx + resize[1]*resize[2]*j + i
	        outimg[ 4*idx    ] = d0;
	        outimg[ 4*idx + 1] = d1;
	        outimg[ 4*idx + 2] = d2;
	        outimg[ 4*idx + 3] = d3;
	        //imgdata[ olen - 4*idx    ] = d0;
	        //imgdata[ olen - 4*idx + 1] = d1;
	        //imgdata[ olen - 4*idx + 2] = d2;
	        //imgdata[ olen - 4*idx + 3] = d3;
              }
	    }
          }
        }
    }

});

