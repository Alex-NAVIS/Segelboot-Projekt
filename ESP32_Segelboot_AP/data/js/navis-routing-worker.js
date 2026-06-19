self.onmessage = function(e) {

    const {
        startNode,
        goalNode,
        routingData
    } = e.data;

    const result = calculateRoute(
        startNode,
        goalNode,
        routingData
    );

    self.postMessage({
        type: "finished",
        route: result
    });
};