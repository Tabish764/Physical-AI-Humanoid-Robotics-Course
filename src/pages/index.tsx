import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="Complete course textbook for Physical AI"
    >
      <main className={styles.heroSection}>
        <h1 className={styles.heroTitle}>Physical AI & Humanoid Robotics</h1>
        <p className={styles.heroSubtitle}>
          Your complete guide to ROS 2, Gazebo, NVIDIA Isaac, Unity, and humanoid robotics.
        </p>
        <Link className={styles.ctaButton} to="/docs">
          Start Reading
        </Link>

        <section className={styles.features}>
          <div className={styles.card}>
            <h3>Module 1: ROS 2</h3>
            <p>Learn nodes, topics, services, and URDF for humanoid robots.</p>
          </div>
          <div className={styles.card}>
            <h3>Module 2: Simulation</h3>
            <p>Gazebo & Unity for physics, sensors, and digital twin simulations.</p>
          </div>
          <div className={styles.card}>
            <h3>Module 3: AI Brain</h3>
            <p>NVIDIA Isaac Sim, Nav2 planning, and Vision-Language-Action integration.</p>
          </div>
          <div className={styles.card}>
            <h3>Module 4: Capstone</h3>
            <p>Build a fully autonomous humanoid robot using learned concepts.</p>
          </div>
        </section>
      </main>
    </Layout>
  );
}
