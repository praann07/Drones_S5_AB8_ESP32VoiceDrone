"""Simple sonification module: converts drone flight state to procedural audio.
Transforms SINDy failure parameters and flight dynamics into real-time sine wave audio.
Used by: Boundary Flight (research - error detection) + Sonification (creative - audio output)
"""

import math
import random
from datetime import datetime


def map_range(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another."""
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)


def sonify_sindy_error(error_data):
    """Convert SINDy error parameters to audio parameters.
    
    Parameters from physics.py's _compute_sindy_error():
    - total: composite error magnitude
    - qr_coupling: roll×yaw rate coupling
    - rp_coupling: yaw×roll coupling  
    - pq_coupling: roll×pitch coupling
    - altitude_divergence: how fast altitude model is diverging
    - thrust_coupling: thrust involvement in dynamics
    - actual_p, actual_q, actual_r: current angular rates
    - est_alt: estimated altitude
    - thrust_norm: normalized thrust (0-1)
    
    Returns dict with audio parameters:
    - frequency: base frequency in Hz for melody
    - amplitude: 0.0-1.0 volume control
    - waveform: 'sine', 'square', 'saw', or 'noise'
    - rhythm: beats per minute for rhythmic pulsation
    - effect: 'none', 'vibrato', 'glissando', 'distortion'
    """
    # Extract key parameters
    total_error = error_data['total']
    qr_coupling = error_data['qr_coupling']
    altitude_div = error_data['altitude_divergence']
    thrust_norm = error_data['thrust_norm']
    est_alt = error_data['est_alt']
    actual_r = error_data['actual_r']
    
    # === FREQUENCY MELODY ===
    # Total error → base melody pitch
    # Higher error = higher/faster melody
    freq_base = map_range(total_error, 0.0, 2.0, 100.0, 800.0)
    
    # QR coupling → adds harmonic interval
    if abs(qr_coupling) > 0.1:
        freq_harmonic = freq_base * (1.0 + map_range(abs(qr_coupling), 0.1, 1.0, 0.5, 1.5))
    else:
        freq_harmonic = freq_base
    
    # Altitude divergence → creates pitch bend/glissando direction
    if altitude_div > 0.5:
        # Diverging fast → upward pitch bend
        freq_bend = freq_harmonic + 200.0
        waveform = 'square'  # square = buzzy, evident bend
    else:
        # Stable → normal sine wave
        freq_bend = freq_harmonic
        waveform = 'sine'
    
    # === AMPLITUDE (VOLUME) ===
    # Thrust norm → volume (more thrust = louder when failing)
    amplitude = map_range(thrust_norm, 0.0, 1.0, 0.3, 0.9)
    
    # Altitude divergence → increase volume when model breaking
    if altitude_div > 1.0:
        amplitude = min(1.0, amplitude + 0.3)
    
    # === RHYTHM (PULSE RATE) ===
    # Angular rate magnitude → pulse speed
    rate_mag = abs(actual_r) * 20  # scale gyro rate to BPM range
    rpm = map_range(rate_mag, 0.0, 5.0, 30.0, 120.0)
    
    # Error total → also affects rhythm intensity
    if total_error > 1.0:
        rpm = min(180.0, rpm * (1.0 + (total_error - 1.0) * 20))
    
    # === EFFECT SELECTION ===
    # High error + high altitude divergence = distortion/vibrato
    if total_error > 1.5 and altitude_div > 1.0:
        effect = 'distortion'  # gritty, broken sound = model failure audified
    elif total_error > 1.0:
        effect = 'vibrato'  # wavering = uncertainty
    else:
        effect = 'none'  # clean sound = stable flight
    
    return {
        'frequency': freq_bend,  # final frequency after bend
        'harmonic_freq': freq_harmonic,
        'amplitude': amplitude,
        'waveform': waveform,
        'rpm': rpm,
        'effect': effect,
        'error_magnitude': total_error,
        'altitude_divergence': altitude_div,
        'is_boundary': total_error > 1.0,
    }


def generate_failure_composition(error_data_list, duration_per_point=0.1):
    """Generate a list of audio samples from a sequence of SINDy error data.
    
    Args:
        error_data_list: List of dicts from sonify_sindy_error()
        duration_per_point: Seconds each point lasts (affects total clip length)
    
    Returns:
        List of (frequency, amplitude, waveform, duration) tuples ready for audio generation
    """
    samples = []
    for error_data in error_data_list:
        params = sonify_sindy_error(error_data)
        params['duration'] = duration_per_point
        samples.append((
            params['frequency'],
            params['amplitude'],
            params['waveform'],
            params['rpm'],
            params['effect'],
        ))
    return samples


def create_boundary_alert_sound():
    """Create a specific audio cue for boundary zone activation.
    
    Returns: (frequency, amplitude, duration) for a 'warning' sound
    when drone enters SINDy model-failure boundary zone.
    """
    # Rising pitch glissando + noisy texture = "warning, model breaking"
    return {
        'frequency': 200.0,  # Start at 200Hz
        'final_frequency': 800.0,  # Glide to 800Hz
        'amplitude': 0.7,
        'duration': 2.0,  # 2-second warning tone
        'waveform': 'sawtooth',
        'effect': 'distortion',
    }


def create_stable_flight_sound():
    """Create audio for stable flight conditions.
    
    Returns: audio parameters for when drone is flying normally
    within SINDy model boundaries.
    """
    return {
        'frequency': 440.0,  # A4 note - stable reference
        'amplitude': 0.2,
        'waveform': 'sine',
        'rpm': 60.0,  # sedate pulse
        'effect': 'none',
    }