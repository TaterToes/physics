import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import javax.swing.*;

public class PhysicsEngine extends JPanel {

    // physics constants
    private static final double GRAVITY = 980.0;
    private static final double DT = 1.0 / 60.0;
    private static final double FRICTION_COEFF = 0.3;
    private static final double ELASTICITY = 0.8;
    private static final double ROTATIONAL_DAMPING = 0.98;

    private static final int RECT_AMOUNT = 10;
    private final List<RectangleObject> rectangles = new ArrayList<>();
    private int windowWidth = 800;
    private int windowHeight = 600;
    private Point mousePos = new Point(0, 0);

    // grid for spatial partitioning
    private static final int GRID_WIDTH = 1000;
    private static final int GRID_HEIGHT = 1000;
    private ArrayList<RectangleObject>[][] grid;

    public PhysicsEngine() {
        Random rand = new Random();
        // initialize the grid (ignore warning)
        grid = new ArrayList[GRID_WIDTH][GRID_HEIGHT];
        for (int i = 0; i < GRID_WIDTH; i++) {
            for (int j = 0; j < GRID_HEIGHT; j++) {
                grid[i][j] = new ArrayList<>();
            }
        }

        int min = 80;  // Lower bound
        int max = 255; // Upper bound

        int width = 100;
        int height = 100;

        for (int i = 0; i < RECT_AMOUNT; i++) {
            int r = rand.nextInt(max - min + 1) + min;
            int g = rand.nextInt(max - min + 1) + min;
            int b = rand.nextInt(max - min + 1) + min;
            Color randomColor = new Color(r, g, b);

            RectangleObject rect = new RectangleObject(i, rand.nextInt(windowWidth), rand.nextInt(windowHeight), 0, width, height, randomColor);
            rectangles.add(rect);
        }

        setPreferredSize(new Dimension(windowWidth, windowHeight));

        // mouse interaction for dragging
        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                if (e.getButton() == MouseEvent.BUTTON1) {
                    Point2D.Double click = new Point2D.Double(e.getX(), e.getY());
                    for (RectangleObject rect : rectangles) {
                        if (rect.contains(click)) {
                            rect.startDragging(click);
                            break;
                        }
                    }
                }
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                if (e.getButton() == MouseEvent.BUTTON1) {
                    for (RectangleObject rect : rectangles) {
                        if (rect.isDragging) {
                            rect.stopDragging();
                        }
                    }
                }
            }
        });
        addMouseMotionListener(new MouseMotionAdapter() {
            @Override
            public void mouseDragged(MouseEvent e) {
                mousePos.setLocation(e.getX(), e.getY());
            }
        });

        Timer timer = new Timer((int) (DT * 1000), e -> {
            updatePhysics();
            repaint();
        });
        timer.start();

        addComponentListener(new ComponentAdapter() {
            @Override
            public void componentResized(ComponentEvent e) {
                windowWidth = getWidth();
                windowHeight = getHeight();
            }
        });
    }

    private void updatePhysics() {
        for (RectangleObject rect : rectangles) {
            rect.update();
        }
        handleRectangleCollisions();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.BLACK);
        g2d.fillRect(0, 0, windowWidth, windowHeight);
        for (RectangleObject rect : rectangles) {
            rect.draw(g2d);
        }
    }

    private void handleRectangleCollisions() {
        // grid clear
        for (int i = 0; i < GRID_WIDTH; i++) {
            for (int j = 0; j < GRID_HEIGHT; j++) {
                grid[i][j].clear();
            }
        }

        // grid size based on window width
        double cellWidth = (double) windowWidth / GRID_WIDTH;
        double cellHeight = (double) windowHeight / GRID_HEIGHT;

        // assign rectangles to grid
        for (RectangleObject rect : rectangles) {
            Rectangle2D.Double aabb = rect.getAABB();
            int minGridX = (int) Math.max(0, Math.floor(aabb.x / cellWidth));
            int maxGridX = (int) Math.min(GRID_WIDTH - 1, Math.floor((aabb.x + aabb.width) / cellWidth));
            int minGridY = (int) Math.max(0, Math.floor(aabb.y / cellHeight));
            int maxGridY = (int) Math.min(GRID_HEIGHT - 1, Math.floor((aabb.y + aabb.height) / cellHeight));
            for (int i = minGridX; i <= maxGridX; i++) {
                for (int j = minGridY; j <= maxGridY; j++) {
                    grid[i][j].add(rect);
                }
            }
        }

        // finds pairs that have potentially collidied
        Set<Pair> potentialPairs = new HashSet<>();
        for (int i = 0; i < GRID_WIDTH; i++) {
            for (int j = 0; j < GRID_HEIGHT; j++) {
                List<RectangleObject> cellRects = grid[i][j];
                for (int k = 0; k < cellRects.size(); k++) {
                    for (int m = k + 1; m < cellRects.size(); m++) {
                        RectangleObject rectA = cellRects.get(k);
                        RectangleObject rectB = cellRects.get(m);
                        potentialPairs.add(new Pair(rectA.index, rectB.index));
                    }
                }
            }
        }

        // does detailed collisions for pairs
        for (Pair p : potentialPairs) {
            RectangleObject rectA = rectangles.get(p.a);
            RectangleObject rectB = rectangles.get(p.b);
            CollisionData collision = rectA.getCollisionData(rectB);
            if (collision != null) {
                // Separate the objects
                double dx = collision.normal.x * collision.penetration / 2;
                double dy = collision.normal.y * collision.penetration / 2;
                rectA.x -= dx;
                rectA.y -= dy;
                rectB.x += dx;
                rectB.y += dy;

                // Resolve velocities
                double relVelX = rectB.vx - rectA.vx;
                double relVelY = rectB.vy - rectA.vy;
                double relNormal = relVelX * collision.normal.x + relVelY * collision.normal.y;
                if (relNormal < 0) {
                    double impulseDenom = (1 / rectA.mass + 1 / rectB.mass);
                    double impulse = -(1 + ELASTICITY) * relNormal / impulseDenom;
                    rectA.vx -= (impulse / rectA.mass) * collision.normal.x;
                    rectA.vy -= (impulse / rectA.mass) * collision.normal.y;
                    rectB.vx += (impulse / rectB.mass) * collision.normal.x;
                    rectB.vy += (impulse / rectB.mass) * collision.normal.y;
                }
            }
        }
    }

    // helper class for unique collision pairs
    private static class Pair {

        int a, b;

        Pair(int a, int b) {
            if (a > b) {
                int temp = a;
                a = b;
                b = temp;
            }
            this.a = a;
            this.b = b;
        }

        @Override
        public boolean equals(Object o) {
            if (o == this) {
                return true;
            }
            if (!(o instanceof Pair)) {
                return false;
            }
            Pair p = (Pair) o;
            return a == p.a && b == p.b;
        }

        @Override
        public int hashCode() {
            return 31 * a + b; // Simple hash function
        }
    }

    // Collision data structure
    private static class CollisionData {

        Point2D.Double normal;
        double penetration;

        CollisionData(Point2D.Double normal, double penetration) {
            this.normal = normal;
            this.penetration = penetration;
        }
    }

    // Rectangle object class
    class RectangleObject {

        public int index;
        double x, y, theta; 
        double vx, vy, omega;
        double width, height, mass, I_cm;
        Point2D.Double grabbedPoint;
        boolean isDragging = false;
        Color color;

        double dragSpring = 150.0;
        double dragDamping = 15.0;

        RectangleObject(int index, double x, double y, double theta, double width, double height, Color color) {
            this.index = index;
            this.x = x;
            this.y = y;
            this.theta = theta;
            this.width = width;
            this.height = height;
            this.mass = 1.0;
            this.I_cm = mass / 12.0 * (width * width + height * height); // Moment of inertia
            this.vx = 0;
            this.vy = 0;
            this.omega = 0;
            this.color = color;
        }

        void update() {
            if (isDragging) {
                // dragging physics with spring and damping
                double cosT = Math.cos(theta);
                double sinT = Math.sin(theta);
                double rX = grabbedPoint.x * cosT - grabbedPoint.y * sinT;
                double rY = grabbedPoint.x * sinT + grabbedPoint.y * cosT;
                double worldGrabX = x + rX;
                double worldGrabY = y + rY;

                double errorX = mousePos.x - worldGrabX;
                double errorY = mousePos.y - worldGrabY;

                double forceX = dragSpring * errorX - dragDamping * (vx - omega * rY);
                double forceY = dragSpring * errorY - dragDamping * (vy + omega * rX);

                vx += (forceX / mass) * DT;
                vy += ((forceY / mass) + GRAVITY) * DT;

                double torque = rX * forceY - rY * forceX;
                omega += (torque / I_cm) * DT;
                omega *= ROTATIONAL_DAMPING;
            } else {
                vy += GRAVITY * DT;
            }

            x += vx * DT;
            y += vy * DT;
            theta += omega * DT;

            handleCollisions();
        }

        void handleCollisions() {
            List<Point2D.Double> vertices = getVertices();
            List<Point2D.Double> floorContacts = new ArrayList<>();

            for (Point2D.Double v : vertices) {
                if (v.y < 0) {
                    y += -v.y;
                    resolveCollisionAtPoint(v, new Point2D.Double(0, 1));
                }
                if (v.x < 0) {
                    x += -v.x;
                    resolveCollisionAtPoint(v, new Point2D.Double(1, 0));
                }
                if (v.x > windowWidth) {
                    x -= v.x - windowWidth;
                    resolveCollisionAtPoint(v, new Point2D.Double(-1, 0));
                }
                if (v.y > windowHeight) {
                    floorContacts.add(v);
                }
            }

            if (!floorContacts.isEmpty()) {
                double sumX = 0, sumY = 0;
                for (Point2D.Double v : floorContacts) {
                    sumX += v.x;
                    sumY += v.y;
                }
                Point2D.Double avgContact = new Point2D.Double(sumX / floorContacts.size(), sumY / floorContacts.size());
                y -= avgContact.y - windowHeight;
                resolveCollisionAtPoint(avgContact, new Point2D.Double(0, -1));

                if (!isDragging) {
                    double frictionForce = FRICTION_COEFF * mass * GRAVITY;
                    double frictionAccel = frictionForce / mass;
                    if (Math.abs(vx) < frictionAccel * DT) {
                        vx = 0;
                    } else {
                        vx -= Math.signum(vx) * frictionAccel * DT;
                    }
                }
            }
        }

        void resolveCollisionAtPoint(Point2D.Double contact, Point2D.Double normal) {
            double rX = contact.x - x;
            double rY = contact.y - y;
            double relVelX = vx - omega * rY;
            double relVelY = vy + omega * rX;
            double relNormal = relVelX * normal.x + relVelY * normal.y;

            if (relNormal < 0) {
                double impulseDenom = (1 / mass) + (Math.pow(rX * normal.y - rY * normal.x, 2) / I_cm);
                double impulse = -(1 + ELASTICITY) * relNormal / impulseDenom;
                vx += (impulse / mass) * normal.x;
                vy += (impulse / mass) * normal.y;
                omega += (impulse * (rX * normal.y - rY * normal.x)) / I_cm;
            }
        }

        List<Point2D.Double> getVertices() {
            List<Point2D.Double> vertices = new ArrayList<>();
            double[][] local = {
                {-width / 2, -height / 2},
                {width / 2, -height / 2},
                {width / 2, height / 2},
                {-width / 2, height / 2}
            };
            double cosT = Math.cos(theta);
            double sinT = Math.sin(theta);
            for (double[] pt : local) {
                double vx = pt[0] * cosT - pt[1] * sinT + x;
                double vy = pt[0] * sinT + pt[1] * cosT + y;
                vertices.add(new Point2D.Double(vx, vy));
            }
            return vertices;
        }

        boolean contains(Point2D.Double p) {
            double dx = p.x - x;
            double dy = p.y - y;
            double cosNegT = Math.cos(-theta);
            double sinNegT = Math.sin(-theta);
            double localX = dx * cosNegT - dy * sinNegT;
            double localY = dx * sinNegT + dy * cosNegT;
            return localX >= -width / 2 && localX <= width / 2 && localY >= -height / 2 && localY <= height / 2;
        }

        void startDragging(Point2D.Double click) {
            double dx = click.x - x;
            double dy = click.y - y;
            double cosNegT = Math.cos(-theta);
            double sinNegT = Math.sin(-theta);
            double localX = dx * cosNegT - dy * sinNegT;
            double localY = dx * sinNegT + dy * cosNegT;
            grabbedPoint = new Point2D.Double(localX, localY);
            isDragging = true;
        }

        void stopDragging() {
            isDragging = false;
        }

        void draw(Graphics2D g2d) {
            AffineTransform at = new AffineTransform();
            at.translate(x, y);
            at.rotate(theta);
            Shape rect = new Rectangle2D.Double(-width / 2, -height / 2, width, height);
            g2d.setColor(color);
            g2d.fill(at.createTransformedShape(rect));
            g2d.setColor(Color.BLACK);
            g2d.draw(at.createTransformedShape(rect));
        }

        CollisionData getCollisionData(RectangleObject other) {
            // Separating Axis Theorem (SAT) for rotated rectangles
            List<Point2D.Double> axes = new ArrayList<>();
            double cosT = Math.cos(theta);
            double sinT = Math.sin(theta);
            axes.add(new Point2D.Double(cosT, sinT));
            axes.add(new Point2D.Double(-sinT, cosT));
            double cosT_other = Math.cos(other.theta);
            double sinT_other = Math.sin(other.theta);
            axes.add(new Point2D.Double(cosT_other, sinT_other));
            axes.add(new Point2D.Double(-sinT_other, cosT_other));

            double minOverlap = Double.POSITIVE_INFINITY;
            Point2D.Double collisionNormal = null;

            for (Point2D.Double axis : axes) {
                double[] projThis = projectOntoAxis(axis);
                double[] projOther = other.projectOntoAxis(axis);
                double overlap = Math.min(projThis[1], projOther[1]) - Math.max(projThis[0], projOther[0]);
                if (overlap <= 0) {
                    return null;
                }
                if (overlap < minOverlap) {
                    minOverlap = overlap;
                    collisionNormal = axis;
                }
            }

            Point2D.Double centerDiff = new Point2D.Double(other.x - x, other.y - y);
            double dot = centerDiff.x * collisionNormal.x + centerDiff.y * collisionNormal.y;
            if (dot < 0) {
                collisionNormal = new Point2D.Double(-collisionNormal.x, -collisionNormal.y);
            }

            return new CollisionData(collisionNormal, minOverlap);
        }

        double[] projectOntoAxis(Point2D.Double axis) {
            List<Point2D.Double> vertices = getVertices();
            double minProj = Double.POSITIVE_INFINITY;
            double maxProj = Double.NEGATIVE_INFINITY;
            for (Point2D.Double v : vertices) {
                double proj = v.x * axis.x + v.y * axis.y;
                minProj = Math.min(minProj, proj);
                maxProj = Math.max(maxProj, proj);
            }
            return new double[]{minProj, maxProj};
        }

        Rectangle2D.Double getAABB() {
            List<Point2D.Double> vertices = getVertices();
            double minX = Double.POSITIVE_INFINITY;
            double minY = Double.POSITIVE_INFINITY;
            double maxX = Double.NEGATIVE_INFINITY;
            double maxY = Double.NEGATIVE_INFINITY;
            for (Point2D.Double v : vertices) {
                minX = Math.min(minX, v.x);
                minY = Math.min(minY, v.y);
                maxX = Math.max(maxX, v.x);
                maxY = Math.max(maxY, v.y);
            }
            return new Rectangle2D.Double(minX, minY, maxX - minX, maxY - minY);
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("physics engine");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            
            frame.add(new PhysicsEngine());
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }
}
