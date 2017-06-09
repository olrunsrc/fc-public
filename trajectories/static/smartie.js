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

    var wSock = new WebSocket("ws://127.0.0.1:9000");
    function sendText(val){
        var msg = { x: val };
        wSock.send(JSON.stringify(msg));
        //wSock.send(val);
        }

    wSock.onmessage = function(event){
        var showData = $('#show-data');
        var msg = JSON.parse(event.data);
        //var msg = (event.data);
        //var msg = {x: event.data};
        showData.empty();
        showData.append("Result received: " + msg.res );
        }

    $('#ws-square').click(function () {
        var val = $('#ws-data').val();
	//alert(val);
        sendText(val);
    });

});

