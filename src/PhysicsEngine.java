import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import javax.swing.*;

public class PhysicsEngine extends JPanel {

    private static final double GRAVITY = 980.0;
    private static final double DT = 1.0 / 60.0;
    private static final double FRICTION_COEFF = 0.3;
    private static final double ELASTICITY = 0.8;
    private static final double ROTATIONAL_DAMPING = 0.98;

    private final List<RectangleObject> rectangles = new ArrayList<>();
    private int windowWidth = 800;
    private int windowHeight = 600;
    private Point mousePos = new Point(0, 0);

    private static final int RECT_AMOUNT = 100;

    public PhysicsEngine() {
        Random rand = new Random();
        for (int i = 0; i < RECT_AMOUNT; i++) {
            RectangleObject rect = new RectangleObject(rand.nextInt(windowWidth), rand.nextInt(windowHeight), 0, 1, 1);
            rectangles.add(rect);
        }

        setPreferredSize(new Dimension(windowWidth, windowHeight));
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
        g2d.setColor(Color.WHITE);
        g2d.fillRect(0, 0, windowWidth, windowHeight);
        for (RectangleObject rect : rectangles) {
            rect.draw(g2d);
        }
    }

    private void handleRectangleCollisions() {
        for (int i = 0; i < rectangles.size(); i++) {
            for (int j = i + 1; j < rectangles.size(); j++) {
                RectangleObject rectA = rectangles.get(i);
                RectangleObject rectB = rectangles.get(j);
                CollisionData collision = rectA.getCollisionData(rectB);
                if (collision != null) {
                    double dx = collision.normal.x * collision.penetration / 2;
                    double dy = collision.normal.y * collision.penetration / 2;
                    rectA.x -= dx;
                    rectA.y -= dy;
                    rectB.x += dx;
                    rectB.y += dy;

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
    }

    private static class CollisionData {
        Point2D.Double normal;
        double penetration;

        CollisionData(Point2D.Double normal, double penetration) {
            this.normal = normal;
            this.penetration = penetration;
        }
    }

    class RectangleObject {

        double x, y, theta;
        double vx, vy, omega;
        double width, height, mass, I_cm;
        Point2D.Double grabbedPoint;
        boolean isDragging = false;

        double dragSpring = 150.0;
        double dragDamping = 15.0;

        RectangleObject(double x, double y, double theta, double width, double height) {
            this.x = x;
            this.y = y;
            this.theta = theta;
            this.width = width;
            this.height = height;
            this.mass = 1.0;
            this.I_cm = mass / 12.0 * (width * width + height * height);
            this.vx = 0;
            this.vy = 0;
            this.omega = 0;
        }

        void update() {
            if (isDragging) {
                double cosT = Math.cos(theta);
                double sinT = Math.sin(theta);
                double rX = grabbedPoint.x * cosT - grabbedPoint.y * sinT;
                double rY = grabbedPoint.x * sinT + grabbedPoint.y * cosT;
                double worldGrabX = x + rX;
                double worldGrabY = y + rY;

                double errorX = mousePos.x - worldGrabX;
                double errorY = mousePos.y - worldGrabY;

                double forceX = dragSpring * errorX;
                double forceY = dragSpring * errorY;

                double grabVelX = vx - omega * rY;
                double grabVelY = vy + omega * rX;
                forceX -= dragDamping * grabVelX;
                forceY -= dragDamping * grabVelY;

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
                    double penetration = -v.y;
                    y += penetration;
                    resolveCollisionAtPoint(v, new Point2D.Double(0, 1));
                }
                if (v.x < 0) {
                    double penetration = -v.x;
                    x += penetration;
                    resolveCollisionAtPoint(v, new Point2D.Double(1, 0));
                }
                if (v.x > windowWidth) {
                    double penetration = v.x - windowWidth;
                    x -= penetration;
                    resolveCollisionAtPoint(v, new Point2D.Double(-1, 0));
                }
                if (v.y > windowHeight) {
                    floorContacts.add(v);
                }
            }

            boolean floorCollision = false;

            if (!floorContacts.isEmpty()) {
                double sumX = 0, sumY = 0;
                for (Point2D.Double v : floorContacts) {
                    sumX += v.x;
                    sumY += v.y;
                }
                double avgX = sumX / floorContacts.size();
                double avgY = sumY / floorContacts.size();
                double penetration = avgY - windowHeight;

                y -= penetration;

                resolveCollisionAtPointReturnImpulse(new Point2D.Double(avgX, avgY), new Point2D.Double(0, -1));
                floorCollision = true;

                if (Math.abs(omega) < 10 && Math.abs(theta % (Math.PI / 2)) < 0.005) {
                    theta = Math.round(theta / (Math.PI / 2)) * (Math.PI / 2);
                    omega = 0;
                }
            }

            if (!isDragging && floorCollision) {
                double frictionForce = FRICTION_COEFF * mass * GRAVITY;
                double frictionAccel = frictionForce / mass;
                if (Math.abs(vx) < frictionAccel * DT) {
                    vx = 0;
                } else {
                    vx -= Math.signum(vx) * frictionAccel * DT;
                }
            }
        }

        void resolveCollisionAtPointReturnImpulse(Point2D.Double contact, Point2D.Double normal) {
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
            for (double[] pt : local) {
                double vx = pt[0] * Math.cos(theta) - pt[1] * Math.sin(theta) + x;
                double vy = pt[0] * Math.sin(theta) + pt[1] * Math.cos(theta) + y;
                vertices.add(new Point2D.Double(vx, vy));
            }
            return vertices;
        }

        boolean contains(Point2D.Double p) {
            double dx = p.x - x;
            double dy = p.y - y;
            double localX = dx * Math.cos(-theta) - dy * Math.sin(-theta);
            double localY = dx * Math.sin(-theta) + dy * Math.cos(-theta);
            return localX >= -width / 2 && localX <= width / 2 && localY >= -height / 2 && localY <= height / 2;
        }

        void startDragging(Point2D.Double click) {
            double dx = click.x - x;
            double dy = click.y - y;
            double localX = dx * Math.cos(-theta) - dy * Math.sin(-theta);
            double localY = dx * Math.sin(-theta) + dy * Math.cos(-theta);
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
            g2d.setColor(Color.BLUE);
            g2d.fill(at.createTransformedShape(rect));
            g2d.setColor(Color.BLACK);
            g2d.draw(at.createTransformedShape(rect));
        }

        CollisionData getCollisionData(RectangleObject other) {
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
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Physics Engine");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.add(new PhysicsEngine());
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }
}