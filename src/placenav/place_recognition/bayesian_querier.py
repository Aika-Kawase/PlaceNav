import numpy as np
import torch
from typing import Tuple

from .gallery_db import PlaceRecognitionDBHandler
from .odometry_motion import OdometryMotionModel


class PlaceRecognitionTopologicalFilter:
    '''
    Adapted from https://github.com/mingu6/ProbFiltersVPR/blob/master/src/models/TopologicalFilter.py
    '''
    def __init__(
        self, extractor, gallery_db_path, gallery_img_dir, delta=5, window_lower=-2, window_upper=10,
        motion_model='legacy', odom_noise=0.1,
    ):        
        self.extractor = extractor

        # load map data
        self.db_handler = PlaceRecognitionDBHandler(gallery_db_path, gallery_img_dir)
        self.descriptors = self.db_handler.get_descriptors()

        # initialize hidden states and obs likelihood parameters
        self.delta = delta
        self.lambda1 = 0.0
        self.belief = None

        # parameters for the transition matrix
        self.window_lower = window_lower
        self.last_diagnostics = None
        self.window_upper = window_upper
        self.window_size = int((window_upper - window_lower) / 2)
        self.transition = np.ones(window_upper - window_lower)
        if motion_model not in ('legacy', 'odometry'):
            raise ValueError('Unknown motion model: ' + motion_model)
        self.motion = None
        if motion_model == 'odometry':
            from pathlib import Path
            self.motion = OdometryMotionModel(
                Path(gallery_img_dir) / 'node_odometry.json',
                self.db_handler.get_filepaths(), noise=odom_noise)

    def initialize_model(self, img):
        '''
        Initialize the belief distribution from the distance of the
        query descriptor to the database descriptors
        
        TODO: Alternatively the initial belief 
              could be concentrated on the start node
        '''
        # Distance between query descriptor and database descriptors
        query_desc = self.extractor(img)['global_descriptor'].numpy().squeeze()
        dists = np.sqrt(np.maximum(0.0, 2 - 2 * np.clip(np.dot(self.descriptors, query_desc), -1, 1)))
        
        # Init for lambda1
        descriptor_quantiles = np.quantile(dists, [0.025, 0.975])
        self.lambda1 = np.log(self.delta) / (
            max(descriptor_quantiles[1] - descriptor_quantiles[0], 1e-6)
        )

        # Init for belief distribution
        self.belief = np.exp(-self.lambda1 * (dists - dists.min()))
        self.belief /= self.belief.sum()
        if self.motion is not None:
            self.motion.initialize(self.belief)
        self.last_diagnostics = None

    def obs_lhood(self, descriptor: np.ndarray) -> np.ndarray:
        '''Observation likelihood of the query descriptor'''
        dists = np.sqrt(np.maximum(0.0, 2 - 2 * np.clip(np.dot(self.descriptors, descriptor), -1, 1)))
        # Preserve positive likelihoods even when a new backend has a very narrow
        # initial distance distribution; otherwise a later update can sum to 0.
        vsim = np.exp(np.maximum(-self.lambda1 * (dists - dists.min()), -80.0))
        return vsim

    def reset_odometry(self):
        if self.motion is not None:
            self.motion.reset_odometry()

    def match(self, img: torch.Tensor, odometry=None) -> Tuple[int, float]:
        '''
        Match the query image to the topological map.

        Runs a prediction step followed by a measurement step:
        - Prediction: Propagate belief mass using the transition model
        - Measurement: Update belief mass using the observation likelihood

        After the process, the map node with the highest probability is
        returned as the subgoal.

        Returns:
        - proposal: the index of the subgoal
        - score: the probability of the subgoal
        '''
        if self.motion is not None:
            if odometry is None:
                raise ValueError('Odometry motion requires an image-time odometry sample')
            previous = self.motion.last_odometry
            if (previous is not None and previous.stamp_ns == odometry.stamp_ns and
                    (previous.frame_id, previous.child_frame_id) ==
                    (odometry.frame_id, odometry.child_frame_id) and
                    self.last_diagnostics is not None):
                proposal = int(np.argmax(self.belief))
                return proposal, float(self.belief[proposal])
        query_desc = self.extractor(img)['global_descriptor'].numpy().squeeze()

        if self.motion is None:
            self._predict_legacy()
        else:
            self.motion.predict(odometry)

        # The VPR likelihood and node-wise Bayesian measurement update are shared.
        cosine_scores = np.dot(self.descriptors, query_desc)
        obs_lhood = self.obs_lhood(query_desc)
        if self.motion is None:
            self.belief *= obs_lhood
            self.belief /= self.belief.sum()
        else:
            self.belief = self.motion.observe(obs_lhood)

        return self._result(cosine_scores)

    def _predict_legacy(self):

        # Calculate the indices of the valid data within the convolution output
        # for the implementation of the prediction step / transition model
        w_l = self.window_lower
        if w_l < 0:
            # In the prediction step, node acquires belief mass
            # from nodes both 'before' and 'after' it in the topomap
            conv_ind_l, conv_ind_h = np.abs(w_l), len(self.belief) + np.abs(w_l)
            bel_ind_l, bel_ind_h = 0, len(self.belief)
        else:
            # Only from nodes 'after' it (and node itself)
            conv_ind_l, conv_ind_h = 0, len(self.belief) - w_l
            bel_ind_l, bel_ind_h = w_l, len(self.belief)

        # Apply transition matrix as a convolution
        # first add symmetric padding to remove edge effects
        # (without it first and last node never reach high values)
        belief_pad = np.pad(self.belief, len(self.transition)-1, mode='symmetric')
        conv = np.convolve( belief_pad, self.transition, mode='valid')

        # Pick the valid part of the convolution output
        self.belief[bel_ind_l:bel_ind_h] = conv[
            conv_ind_l:conv_ind_h
        ]

        if w_l > 0:
            self.belief[:w_l] = 0.0

    def _result(self, cosine_scores):
        # Argmax of the belief
        max_bel = np.argmax(self.belief)
        score = self.belief[max_bel]
        proposal = max_bel
        
        ranked = np.argsort(cosine_scores)[::-1]
        top1_idx = int(ranked[0])
        top2_idx = int(ranked[1]) if len(ranked) > 1 else top1_idx
        top1_score = float(cosine_scores[top1_idx])
        top2_score = float(cosine_scores[top2_idx])
        node_indices = np.arange(len(self.belief), dtype=np.float64)
        belief_mean = float(np.sum(node_indices * self.belief))
        belief_std = float(np.sqrt(np.sum(
            ((node_indices - belief_mean) ** 2) * self.belief
        )))
        nonzero_belief = self.belief[self.belief > 0]
        belief_entropy = float(-np.sum(nonzero_belief * np.log(nonzero_belief)))
        if len(self.belief) > 1:
            belief_entropy /= float(np.log(len(self.belief)))

        self.last_diagnostics = {
            "cosplace_top1_node_idx": top1_idx,
            "cosplace_top2_node_idx": top2_idx,
            "cosplace_top1_score": top1_score,
            "cosplace_top2_score": top2_score,
            "cosplace_score_margin": top1_score - top2_score,
            "bayesian_belief_max": float(score),
            "bayesian_belief_std": belief_std,
            "bayesian_belief_entropy": belief_entropy,
        }

        return proposal, score
