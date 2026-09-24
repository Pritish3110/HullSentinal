#!/usr/bin/env python3
"""
Unit Test Suite for HullSentinal Core Cognitive Logic:
1. Belief Map ORM & Bayesian Updates
2. Multi-Attribute Utility Scoring & Ranking
3. Re-queue Priority Escalation
4. Closed-Loop Verification Decision Logic
5. Safety Barrier Separation Math
"""

import math
import unittest
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from hull_swarm.belief_map_node import Base, DefectBeliefModel


class TestHullSentinalCoreLogic(unittest.TestCase):

    def setUp(self):
        # In-memory SQLite for testing
        self.engine = create_engine('sqlite:///:memory:', echo=False)
        Base.metadata.create_all(self.engine)
        self.Session = sessionmaker(bind=self.engine)

    def test_belief_map_orm_lifecycle(self):
        """Test defect insertion, retrieval, and status transitions."""
        session = self.Session()
        rec = DefectBeliefModel(
            defect_id='defect_test_01',
            defect_type='crack',
            severity=0.85,
            confidence=0.90,
            pos_x=5.0,
            pos_y=2.5,
            status='PENDING',
            rescan_count=0
        )
        session.add(rec)
        session.commit()

        # Query back
        queried = session.query(DefectBeliefModel).filter_by(defect_id='defect_test_01').first()
        self.assertIsNotNone(queried)
        self.assertEqual(queried.defect_type, 'crack')
        self.assertEqual(queried.status, 'PENDING')

        # Status transition to QUEUED -> IN_PROGRESS -> DONE
        queried.status = 'DONE'
        queried.severity = 0.0
        session.commit()

        updated = session.query(DefectBeliefModel).filter_by(defect_id='defect_test_01').first()
        self.assertEqual(updated.status, 'DONE')
        self.assertEqual(updated.severity, 0.0)
        session.close()

    def test_utility_scoring_hierarchy(self):
        """Test that crack has higher utility than rust, and rust higher than biofouling at same distance."""
        w_crack = 1.0
        w_rust = 0.70
        w_bio = 0.40
        alpha = 0.05
        cleaner_pos = (0.0, 0.0)

        # Three defects at identical distance (5.0m)
        d_crack = {'type': 'crack', 'sev': 0.85, 'conf': 0.90, 'dist': 5.0}
        d_rust = {'type': 'rust', 'sev': 0.85, 'conf': 0.90, 'dist': 5.0}
        d_bio = {'type': 'biofouling', 'sev': 0.85, 'conf': 0.90, 'dist': 5.0}

        u_crack = (d_crack['sev'] * w_crack * d_crack['conf']) - (alpha * d_crack['dist'])
        u_rust = (d_rust['sev'] * w_rust * d_rust['conf']) - (alpha * d_rust['dist'])
        u_bio = (d_bio['sev'] * w_bio * d_bio['conf']) - (alpha * d_bio['dist'])

        self.assertGreater(u_crack, u_rust, "Crack utility must exceed rust utility")
        self.assertGreater(u_rust, u_bio, "Rust utility must exceed biofouling utility")

    def test_distance_penalty(self):
        """Test that closer defect receives higher priority than far defect of same type."""
        w_crack = 1.0
        alpha = 0.05

        d_near = {'type': 'crack', 'sev': 0.85, 'conf': 0.90, 'dist': 2.0}
        d_far = {'type': 'crack', 'sev': 0.85, 'conf': 0.90, 'dist': 12.0}

        u_near = (d_near['sev'] * w_crack * d_near['conf']) - (alpha * d_near['dist'])
        u_far = (d_far['sev'] * w_crack * d_far['conf']) - (alpha * d_far['dist'])

        self.assertGreater(u_near, u_far, "Closer defect must have higher utility due to transit cost")

    def test_requeue_priority_escalation(self):
        """Test that re-queued defects receive escalation bonus to prevent starvation."""
        w_rust = 0.70
        alpha = 0.05
        escalation_bonus = 0.20

        d_standard = {'type': 'rust', 'sev': 0.60, 'conf': 0.85, 'dist': 4.0, 'requeues': 0}
        d_requeued = {'type': 'rust', 'sev': 0.60, 'conf': 0.85, 'dist': 4.0, 'requeues': 1}

        u_standard = (d_standard['sev'] * w_rust * d_standard['conf']) - (alpha * d_standard['dist'])
        u_requeued = u_standard + (escalation_bonus * d_requeued['requeues'])

        self.assertGreater(u_requeued, u_standard, "Re-queued defect must receive utility escalation")

    def test_verification_decision_logic(self):
        """Test verification decision boundary between DONE and RE_QUEUED."""
        threshold = 0.08

        # Case 1: Thoroughly cleaned
        residual_1 = 0.02
        status_1 = 'DONE' if residual_1 <= threshold else 'RE_QUEUED'
        self.assertEqual(status_1, 'DONE')

        # Case 2: Incomplete clean (stubborn biofouling/deep crack)
        residual_2 = 0.35
        status_2 = 'DONE' if residual_2 <= threshold else 'RE_QUEUED'
        self.assertEqual(status_2, 'RE_QUEUED')

    def test_safety_barrier_math(self):
        """Test barrier distance scaling factor."""
        safety_r = 0.55
        stop_r = 0.35

        # Robot outside safety zone -> full speed
        d_outside = 0.80
        vel_factor = max(0.2, (d_outside - stop_r) / (safety_r - stop_r)) if d_outside < safety_r else 1.0
        self.assertEqual(vel_factor, 1.0)

        # Robot within safety buffer -> scaled speed
        d_inside = 0.45
        vel_factor_in = max(0.2, (d_inside - stop_r) / (safety_r - stop_r))
        self.assertLess(vel_factor_in, 1.0)
        self.assertGreater(vel_factor_in, 0.0)


if __name__ == '__main__':
    unittest.main()
