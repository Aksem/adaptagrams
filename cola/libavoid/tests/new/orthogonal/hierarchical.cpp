#include "libavoid/libavoid.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "helpers.h"

/*
 * Test routing between child shapes in a parent shape.
 * All child shapes have two or four shape connection pins(with the same class id per pair) on shape edges for
 * outgoing connections.
 * */


const int CONNECTIONPIN_INCOMING = 111;

class HierarchicalOrthogonalRouter : public ::testing::Test {
protected:
    void SetUp() override {
        router = new Avoid::Router(Avoid::OrthogonalRouting);
        router->setRoutingParameter(Avoid::RoutingParameter::shapeBufferDistance, 4);
        router->setRoutingParameter(Avoid::RoutingParameter::segmentPenalty, 50);
        router->setRoutingParameter(Avoid::RoutingParameter::idealNudgingDistance, 4);
        router->setRoutingOption(Avoid::RoutingOption::nudgeOrthogonalSegmentsConnectedToShapes, true);
        router->setRoutingOption(Avoid::RoutingOption::nudgeOrthogonalTouchingColinearSegments, true);

        Avoid::Rectangle parent({ -100, -400 }, { 900, 900 });
        new Avoid::ShapeRef(router, parent, 1);
    }

    void TearDown() override {
        delete router;
    }

    Avoid::ShapeRef* addChild(Avoid::Point topLeft, Avoid::Point bottomRight, unsigned int shapeId, unsigned int connectionId, unsigned int connectionId2 = 0) {
        Avoid::Rectangle childRectangle(topLeft, bottomRight);
        Avoid::ShapeRef *childShape = new Avoid::ShapeRef(router, childRectangle, shapeId);
        new Avoid::ShapeConnectionPin(childShape, connectionId, 0, 14, false, 0, Avoid::ConnDirLeft);
        new Avoid::ShapeConnectionPin(childShape, connectionId, 200, 14, false, 0, Avoid::ConnDirRight);
        if (connectionId2 != 0) {
            new Avoid::ShapeConnectionPin(childShape, connectionId2, 0, 56, false, 0, Avoid::ConnDirLeft);
            new Avoid::ShapeConnectionPin(childShape, connectionId2, 200, 56, false, 0, Avoid::ConnDirRight);
        }
        new Avoid::ShapeConnectionPin(childShape, Avoid::CONNECTIONPIN_CENTRE,
                                      Avoid::ATTACH_POS_CENTRE, Avoid::ATTACH_POS_CENTRE, true, 0.0, Avoid::ConnDirAll);
        return childShape;
    }

    Avoid::ShapeRef* addChildWithIncomingPins(Avoid::Point topLeft, Avoid::Point bottomRight, unsigned int shapeId, unsigned int connectionId, unsigned int connectionId2 = 0) {
        Avoid::Rectangle childRectangle(topLeft, bottomRight);
        Avoid::ShapeRef *childShape = new Avoid::ShapeRef(router, childRectangle, shapeId);
        new Avoid::ShapeConnectionPin(childShape, connectionId, 0, 14, false, 0, Avoid::ConnDirLeft);
        new Avoid::ShapeConnectionPin(childShape, connectionId, 200, 14, false, 0, Avoid::ConnDirRight);
        if (connectionId2 != 0) {
            new Avoid::ShapeConnectionPin(childShape, connectionId2, 0, 56, false, 0, Avoid::ConnDirLeft);
            new Avoid::ShapeConnectionPin(childShape, connectionId2, 200, 56, false, 0, Avoid::ConnDirRight);
        }
        new Avoid::ShapeConnectionPin(childShape, CONNECTIONPIN_INCOMING,
                               0, 150, false, 0.0, Avoid::ConnDirLeft);
        new Avoid::ShapeConnectionPin(childShape, CONNECTIONPIN_INCOMING,
                               200, 150, false, 0.0, Avoid::ConnDirRight);
        return childShape;
    }

    Avoid::ConnRef*  connectShapes(Avoid::ShapeRef *shape1, unsigned int shape1ConnId, Avoid::ShapeRef *shape2, unsigned int shape2ConnId) {
        Avoid::ConnEnd srcPtEnd(shape1, shape1ConnId);
        Avoid::ConnEnd dstPtEnd(shape2, shape2ConnId);
        Avoid::ConnRef *connection = new Avoid::ConnRef(router, srcPtEnd, dstPtEnd);
        return connection;
    }

    Avoid::Router *router;
};

/* Checks: https://github.com/Aksem/adaptagrams/issues/3 */
TEST_F(HierarchicalOrthogonalRouter, TwoChildrenVertically) {
    // two children connected from pins on border to center
    Avoid::ShapeRef *topChildShape = addChild({ 600, 500 }, { 800, 700 }, 2, 5);
    Avoid::ShapeRef *bottomChildShape = addChild({ 600, 700 }, { 800, 900 }, 3, 6);

    Avoid::ConnRef *bottomToTopConn = connectShapes(bottomChildShape, 6, topChildShape, Avoid::CONNECTIONPIN_CENTRE);
    Avoid::ConnRef *topToBottomConn = connectShapes(topChildShape, 5, bottomChildShape, Avoid::CONNECTIONPIN_CENTRE);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HierarchicalOrthogonalRouter_TwoChildrenVertically");

    std::vector<Avoid::Point> expectedBottomToTop = { {600, 714}, {596, 714}, {596, 600}, {700, 600} };
    EXPECT_THAT(bottomToTopConn, IsEqualToRoute(expectedBottomToTop));
    std::vector<Avoid::Point> expectedTopToBottom = { {600, 514}, {592, 514}, {592, 800}, {700, 800} };
    EXPECT_THAT(topToBottomConn, IsEqualToRoute(expectedTopToBottom));
}

TEST_F(HierarchicalOrthogonalRouter, TwoChildrenVerticallyAllWithPins) {
    // two vertically positioned shapes connected from pins on borders to pins
    // on borders there are two pairs of connections to test both sides,
    // issues like https://github.com/Aksem/adaptagrams/issues/9 can
    // appear also on one side(e.g. closer to parent side)
    Avoid::ShapeRef *topChildShape = addChildWithIncomingPins({ 650, 200 }, { 850, 400 }, 2, 5, 6);
    Avoid::ShapeRef *bottomChildShape = addChildWithIncomingPins({ 650, 500 }, { 850, 700 }, 3, 7, 8);

    Avoid::ConnRef *bottomToTopConn = connectShapes(bottomChildShape, 7, topChildShape, CONNECTIONPIN_INCOMING);
    Avoid::ConnRef *bottomToTopConn2 = connectShapes(bottomChildShape, 8, topChildShape, CONNECTIONPIN_INCOMING);
    Avoid::ConnRef *topToBottomConn = connectShapes(topChildShape, 5, bottomChildShape, CONNECTIONPIN_INCOMING);
    Avoid::ConnRef *topToBottomConn2 = connectShapes(topChildShape, 6, bottomChildShape, CONNECTIONPIN_INCOMING);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HierarchicalOrthogonalRouter_TwoChildrenVerticallyAllWithPins");

    std::vector<Avoid::Point> expectedBottomToTop = { {850, 514}, {854, 514}, {854, 350}, {850, 350} };
    EXPECT_THAT(bottomToTopConn, IsEqualToRoute(expectedBottomToTop));
    std::vector<Avoid::Point> expectedBottomToTop2 = { {650, 556}, {646, 556}, {646, 350}, {650, 350} };
    EXPECT_THAT(bottomToTopConn2, IsEqualToRoute(expectedBottomToTop2));
    std::vector<Avoid::Point> expectedTopToBottom = { {850, 214}, {858, 214}, {858, 650}, {850, 650} };
    EXPECT_THAT(topToBottomConn, IsEqualToRoute(expectedTopToBottom));
    std::vector<Avoid::Point> expectedTopToBottom2 = { {650, 256}, {642, 256}, {642, 650}, {650, 650} };
    EXPECT_THAT(topToBottomConn2, IsEqualToRoute(expectedTopToBottom2));
}


TEST_F(HierarchicalOrthogonalRouter, ThreeChildrenVertically) {
    Avoid::ShapeRef *topChildShape = addChild({ 600, 300 }, { 800, 500 }, 2, 5);
    Avoid::ShapeRef *bottomChildShape = addChild({ 600, 600 }, { 800, 800 }, 3, 6);
    Avoid::ShapeRef *leftChildShape = addChild({100, 400}, {300, 600}, 4, 7, 8);

    Avoid::ConnRef *bottomToTopConn = connectShapes(bottomChildShape, 6, topChildShape, Avoid::CONNECTIONPIN_CENTRE);
    Avoid::ConnRef *topToBottomConn = connectShapes(topChildShape, 5, bottomChildShape, Avoid::CONNECTIONPIN_CENTRE);
    Avoid::ConnRef *leftToTopConn = connectShapes(leftChildShape, 7, topChildShape, Avoid::CONNECTIONPIN_CENTRE);
    Avoid::ConnRef *leftToBottomConn = connectShapes(leftChildShape, 8, bottomChildShape, Avoid::CONNECTIONPIN_CENTRE);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HierarchicalOrthogonalRouter_ThreeChildrenVertically");

    std::vector<Avoid::Point> expectedBottomToTop = { {600, 614}, {596, 614}, {596, 400}, {700, 400} };
    EXPECT_THAT(bottomToTopConn, IsEqualToRoute(expectedBottomToTop));
    std::vector<Avoid::Point> expectedTopToBottom = { {600, 314}, {592, 314}, {592, 700}, {700, 700} };
    EXPECT_THAT(topToBottomConn, IsEqualToRoute(expectedTopToBottom));
    std::vector<Avoid::Point> expectedLeftToTop = { {300, 407}, {700, 407} };
    EXPECT_THAT(leftToTopConn, IsEqualToRoute(expectedLeftToTop));
    std::vector<Avoid::Point> expectedLeftToBottom = { {300, 600}, {700, 600} };
    EXPECT_THAT(leftToBottomConn, IsEqualToRoute(expectedLeftToBottom));
}
