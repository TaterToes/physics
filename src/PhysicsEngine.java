
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.ArrayList;
import java.util.List;
import javax.swing.*;

public class PhysicsEngine extends JPanel {

    private static final double GRAVITY = 980.0;
    private static final double DT = 1.0 / 60.0;
    private static final double FRICTION_COEFF = 0.3;
    private static final double ELASTICITY = 0.8;
    private static final double ROTATIONAL_DAMPING = 0.98;

    private final List<RectangleObject> rectangles = new ArrayList<>();
    private RectangleObject rectangle;
    private int windowWidth = 800;
    private int windowHeight = 600;
    private Point mousePos = new Point(0, 0);

    public PhysicsEngine() {
        rectangle = new RectangleObject(400, 300, 0, 100, 50);
        rectangles.add(rectangle);

        setPreferredSize(new Dimension(windowWidth, windowHeight));
        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                if (e.getButton() == MouseEvent.BUTTON1) {
                    mousePos.setLocation(e.getX(), e.getY());
                    Point2D.Double click = new Point2D.Double(e.getX(), e.getY());
                    if (rectangle.contains(click)) {
                        rectangle.startDragging(click);
                    }
                }
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                if (e.getButton() == MouseEvent.BUTTON1 && rectangle.isDragging) {
                    rectangle.stopDragging();
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

    class RectangleObject {

        double x, y, theta; // center position and rotation angle
        double vx, vy, omega; // linear and angular velocities
        double width, height, mass, I_cm;
        Point2D.Double grabbedPoint; // local coordinate of the grabbed point
        boolean isDragging = false;

        // Spring/damping constants for dragging
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
                // Calculate the current world coordinates of the grabbed point
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

                // apply force to update center of mass
                vx += (forceX / mass) * DT;
                vy += ((forceY / mass) + GRAVITY) * DT; // gravity still applies

                // compute torque from the force relative to the center
                double torque = rX * forceY - rY * forceX;
                omega += (torque / I_cm) * DT;

                // apply damping to the angular velocity
                omega *= ROTATIONAL_DAMPING;
            } else {
                // add velocity
                vy += GRAVITY * DT;
            }

            // update position and rotation
            x += vx * DT;
            y += vy * DT;
            theta += omega * DT;

            // Handle collisions
            handleCollisions();
        }

        void handleCollisions() {
            List<Point2D.Double> vertices = getVertices();
            List<Point2D.Double> floorContacts = new ArrayList<>();

            for (Point2D.Double v : vertices) {
                // top
                if (v.y < 0) {
                    double penetration = 0 - v.y;
                    y += penetration;
                    resolveCollisionAtPoint(v, new Point2D.Double(0, 1));
                }
                // Left wall
                if (v.x < 0) {
                    double penetration = 0 - v.x;
                    x += penetration;
                    resolveCollisionAtPoint(v, new Point2D.Double(1, 0));
                }
                // Right wall
                if (v.x > windowWidth) {
                    double penetration = v.x - windowWidth;
                    x -= penetration;
                    resolveCollisionAtPoint(v, new Point2D.Double(-1, 0));
                }
                // add rectangle contacts with floor
                if (v.y > windowHeight) {
                    floorContacts.add(v);
                }
            }

            boolean floorCollision = false;

            // if it touches floor
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

                // if theta, impulse, and omega are small, just stop it from jiggling.
                if (Math.abs(omega) < 10
                        && Math.abs(theta % (Math.PI / 2)) < 0.005) {
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
            // vector from the center to the contact point.
            double rX = contact.x - x;
            double rY = contact.y - y;

            // relative velocity at the contact point
            double relVelX = vx - omega * rY;
            double relVelY = vy + omega * rX;

            // component of relative velocity along the normal.
            double relNormal = relVelX * normal.x + relVelY * normal.y;

            double impulse = 0;

            if (relNormal < 0) {
                double impulseDenom = (1 / mass) + (Math.pow(rX * normal.y - rY * normal.x, 2) / I_cm);
                impulse = -(1 + ELASTICITY) * relNormal / impulseDenom;

                // apply impulse to linear velocity.
                vx += (impulse / mass) * normal.x;
                vy += (impulse / mass) * normal.y;

                // apply impulse to angular velocity.
                omega += (impulse * (rX * normal.y - rY * normal.x)) / I_cm;
            }
        }

        // regular collision
        void resolveCollisionAtPoint(Point2D.Double contact, Point2D.Double normal) {
            // vector from the center to the contact point.
            double rX = contact.x - x;
            double rY = contact.y - y;

            // relative velocity at the contact point.
            double relVelX = vx - omega * rY;
            double relVelY = vy + omega * rX;

            // component of relative velocity along the collision normal.
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
            // Convert click into local coordinates.
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
